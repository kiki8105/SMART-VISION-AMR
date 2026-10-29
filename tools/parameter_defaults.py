"""Persist confirmed ROS parameter edits into explicitly bound launch YAML files.

The YAML syntax tree supplies exact source ranges, so changing a value does not
reformat unrelated settings or discard their comments. No ROS or Qt import is
needed here; the controller supplies only changes accepted by a parameter service.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
from uuid import uuid4

import yaml
from yaml.nodes import MappingNode, ScalarNode, SequenceNode
from yaml.tokens import AliasToken, AnchorToken, ScalarToken


_RESERVED_PARAMETERS = frozenset({
    'drive_enabled', 'autostart', 'use_sim_time', 'yaml_filename',
    'set_initial_pose', 'memory_enabled', 'memory_path', 'robot_description',
    'node_names',
})


class ParameterDefaultsError(ValueError):
    """A defaults file could not be safely matched, updated, or committed."""


def is_persistent_parameter(name):
    """Whether a parameter is tuning rather than launch-managed/runtime state."""
    return (isinstance(name, str) and bool(name)
            and name not in _RESERVED_PARAMETERS
            and name.rsplit('.', 1)[-1] not in {'drive_enabled', 'autostart', 'use_sim_time'}
            and name != 'initial_pose' and not name.startswith('initial_pose.'))


def _node_name(name):
    if not isinstance(name, str) or not name.strip('/') or any(c.isspace() for c in name):
        raise ParameterDefaultsError(f'올바르지 않은 노드 이름: {name!r}')
    return '/' + '/'.join(part for part in name.split('/') if part)


def _parameter_name(name):
    if (not isinstance(name, str) or not name or name != name.strip()
            or any(c in name for c in '\r\n\0')
            or any(not part for part in name.split('.'))):
        raise ParameterDefaultsError(f'올바르지 않은 파라미터 이름: {name!r}')
    return name


def _parameter_value(value):
    if type(value) in (str, bool, int):
        if type(value) is int and not -(2 ** 63) <= value < 2 ** 63:
            raise ParameterDefaultsError('ROS 정수 범위(-2^63 … 2^63-1)를 벗어났습니다.')
        return value
    if type(value) is float and math.isfinite(value):
        return value
    if isinstance(value, (list, tuple)):
        values = [_parameter_value(item) for item in value]
        if any(isinstance(item, list) for item in values):
            raise ParameterDefaultsError('중첩 배열은 ROS 파라미터로 저장할 수 없습니다.')
        if len({type(item) for item in values}) > 1:
            raise ParameterDefaultsError('ROS 파라미터 배열의 모든 값은 같은 형식이어야 합니다.')
        return values
    raise ParameterDefaultsError('숫자, bool, 문자열 또는 같은 형식의 배열만 저장할 수 있습니다.')


def _changes(changes, persistent):
    confirmed = {}
    for key, value in changes.items():
        if not isinstance(key, tuple) or len(key) != 2:
            raise ParameterDefaultsError('변경값 키는 (노드, 파라미터)여야 합니다.')
        node, name = _node_name(key[0]), _parameter_name(key[1])
        if persistent and not is_persistent_parameter(name):
            raise ParameterDefaultsError(f'실행 상태/launch 관리값은 기본값으로 저장할 수 없습니다: {node} / {name}')
        if (node, name) in confirmed:
            raise ParameterDefaultsError(f'중복 변경값: {node} / {name}')
        confirmed[node, name] = _parameter_value(value)
    return confirmed


def _key(node):
    if not isinstance(node, ScalarNode) or node.tag != 'tag:yaml.org,2002:str':
        raise ParameterDefaultsError('ROS YAML의 모든 설정 키는 문자열이어야 합니다.')
    return node.value


def _check_mapping_keys(node):
    if isinstance(node, MappingNode):
        seen = set()
        for key, value in node.value:
            name = _key(key)
            if name in seen:
                raise ParameterDefaultsError(f'YAML에 중복 키가 있습니다: {name}')
            seen.add(name)
            _check_mapping_keys(value)
    elif isinstance(node, SequenceNode):
        for value in node.value:
            _check_mapping_keys(value)


def _node_sections(root):
    sections = {}

    def visit(mapping, components):
        if not isinstance(mapping, MappingNode):
            return
        parameters = [value for key, value in mapping.value if _key(key) == 'ros__parameters']
        if parameters:
            if not components or not isinstance(parameters[0], MappingNode):
                raise ParameterDefaultsError('ros__parameters는 노드 아래의 YAML mapping이어야 합니다.')
            name = _node_name('/'.join(components))
            if name in sections:
                raise ParameterDefaultsError(f'같은 노드가 YAML에 여러 번 정의되어 있습니다: {name}')
            sections[name] = parameters[0]
            return
        for key, value in mapping.value:
            visit(value, components + [part for part in _key(key).split('/') if part])

    visit(root, [])
    return sections


def _parameter_locations(mapping):
    locations = {}

    def visit(section, prefix):
        for key, value in section.value:
            name = '.'.join(prefix + [_key(key)])
            if isinstance(value, MappingNode):
                visit(value, prefix + [_key(key)])
            else:
                if name in locations:
                    raise ParameterDefaultsError(f'같은 파라미터의 중복 dotted/nested 표현: {name}')
                locations[name] = value

    visit(mapping, [])
    return locations


@dataclass
class _Config:
    path: Path
    raw: bytes
    text: str
    mode: int
    sections: dict
    locations: dict
    scalar_ranges: list


def _parse(path, raw, mode):
    try:
        text = raw.decode('utf-8')
        tokens = list(yaml.scan(text, Loader=yaml.SafeLoader))
        if any(isinstance(token, (AliasToken, AnchorToken)) for token in tokens):
            raise ParameterDefaultsError('YAML anchor/alias가 있는 파일은 다른 값에 영향을 줄 수 있어 저장할 수 없습니다.')
        root = yaml.compose(text, Loader=yaml.SafeLoader)
        if not isinstance(root, MappingNode):
            raise ParameterDefaultsError('ROS 파라미터 파일은 YAML mapping이어야 합니다.')
        _check_mapping_keys(root)
        # Reject unsupported tags and malformed values before preparing any write.
        yaml.safe_load(text)
        sections = _node_sections(root)
        locations = {node: _parameter_locations(section) for node, section in sections.items()}
        return _Config(path, raw, text, mode, sections, locations,
                       [(token.start_mark.index, token.end_mark.index)
                        for token in tokens if isinstance(token, ScalarToken)])
    except (yaml.YAMLError, UnicodeError) as exc:
        raise ParameterDefaultsError(f'YAML을 읽을 수 없습니다: {path}: {exc}') from exc


def _read(path):
    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ParameterDefaultsError(f'설정 파일이 아닙니다: {path}')
    return _parse(path, path.read_bytes(), stat.S_IMODE(path.stat().st_mode))


def _flow_value(value):
    # JSON quoting keeps strings (including multiline strings) on one YAML line.
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, list):
        return '[' + ', '.join(_flow_value(item) for item in value) + ']'
    dumped = yaml.safe_dump(value, default_flow_style=True, allow_unicode=True, width=100000).rstrip('\n')
    return dumped[:-4] if dumped.endswith('\n...') else dumped


def _comments_in(config, start, end):
    comments = []
    position = start
    while position < end:
        position = config.text.find('#', position, end)
        if position < 0:
            break
        if any(left <= position < right for left, right in config.scalar_ranges):
            position += 1
            continue
        line_end = config.text.find('\n', position, end)
        if line_end < 0:
            line_end = end
        comments.append(config.text[position:line_end].rstrip('\r'))
        position = line_end + 1
    return comments


def _replace_value(config, node, value):
    start, end = node.start_mark.index, node.end_mark.index
    original = config.text[start:end]
    newline = '\r\n' if '\r\n' in config.text else '\n'
    replacement = _flow_value(value)
    # Block arrays can contain comments inside the replaced value range. Keep
    # these as adjacent comment lines, rather than losing them with the old items.
    for comment in _comments_in(config, start, end):
        replacement += newline + ' ' * node.start_mark.column + comment
    trailing = re.search(r'(?:\r?\n)[ \t\r\n]*$', original)
    if trailing:
        replacement += trailing.group()
    return start, end, replacement


def _add_values(config, section, additions):
    entries = [json.dumps(name, ensure_ascii=False) + ': ' + _flow_value(value)
               for name, value in additions]
    if section.flow_style:
        position = section.end_mark.index - 1
        return position, position, (', ' if section.value else '') + ', '.join(entries)
    newline = '\r\n' if '\r\n' in config.text else '\n'
    # A block mapping ends at the next key's indentation, or at EOF.
    if section.end_mark.index == len(config.text):
        position = len(config.text)
    else:
        position = section.end_mark.index - section.end_mark.column
    prefix = '' if not position or config.text[position - 1] == '\n' else newline
    insertion = prefix + ''.join(' ' * section.start_mark.column + entry + newline for entry in entries)
    return position, position, insertion


def _snapshot(config):
    loader = yaml.SafeLoader('')
    try:
        return {(node, name): loader.construct_object(value, deep=True)
                for node, values in config.locations.items() for name, value in values.items()}
    finally:
        loader.dispose()


def _render(config, changes):
    edits, additions = [], {}
    for (node, name), value in changes.items():
        if node not in config.sections:
            raise ParameterDefaultsError(f'기준 YAML에 노드 {node} 없음: {config.path}')
        location = config.locations[node].get(name)
        if location is None:
            additions.setdefault(node, []).append((name, value))
        else:
            edits.append(_replace_value(config, location, value))
    for node, values in additions.items():
        edits.append(_add_values(config, config.sections[node], values))
    text = config.text
    for start, end, value in sorted(edits, key=lambda edit: (edit[0], edit[1]), reverse=True):
        text = text[:start] + value + text[end:]
    rendered = _parse(config.path, text.encode('utf-8'), config.mode)
    expected = _snapshot(config)
    expected.update(changes)
    if _snapshot(rendered) != expected:
        raise ParameterDefaultsError(f'저장 결과가 요청한 변경값과 다릅니다. 파일은 수정하지 않았습니다: {config.path}')
    return rendered.raw


def _stage(path, raw, mode):
    descriptor, filename = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.tmp', dir=path.parent)
    staged = Path(filename)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            os.fchmod(stream.fileno(), mode)
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        return staged
    except BaseException:
        staged.unlink(missing_ok=True)
        raise


def _backup(config):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    backup = config.path.with_name(config.path.name + '.bak-' + stamp + '-' + uuid4().hex[:8])
    # The generated name is unique; exclusive creation still avoids overwriting
    # a pre-existing backup if a filename is externally reserved in the meantime.
    with backup.open('xb') as stream:
        stream.write(config.raw)
        stream.flush()
        os.fsync(stream.fileno())
    shutil.copystat(config.path, backup)
    return backup


def _commit(configs, outputs, changes=None):
    staged, backups, replaced = {}, {}, []
    receipt = None
    try:
        for config in configs:
            if outputs[config.path] != config.raw:
                staged[config.path] = _stage(config.path, outputs[config.path], config.mode)
        # Validate every original again before writing any source or backup.
        for config in configs:
            if config.path.read_bytes() != config.raw:
                raise ParameterDefaultsError(f'조회 후 설정 파일이 변경되었습니다. 다시 저장하세요: {config.path}')
        for config in configs:
            if config.path in staged:
                backups[config.path] = _backup(config)
        if staged:
            history_dir = configs[0].path.parent / '.parameter-history'
            history_dir.mkdir(mode=0o700, exist_ok=True)
            receipt = history_dir / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+uuid4().hex[:8]+'.json')
            record = {'version': 1, 'status': 'prepared', 'at': datetime.now(timezone.utc).isoformat(),
                      'changes': [{'node': node, 'parameter': name, 'value': value}
                                  for (node, name), value in (changes or {}).items()],
                      'files': [{'path': str(config.path), 'backup': str(backups[config.path]),
                                 'before_sha256': hashlib.sha256(config.raw).hexdigest(),
                                 'after_sha256': hashlib.sha256(outputs[config.path]).hexdigest()}
                                for config in configs if config.path in staged]}
            for change in record['changes']:
                source = next(config for config in configs if change['node'] in config.sections)
                previous = _parameter_locations(source.sections[change['node']]).get(change['parameter'])
                change['previous_value'] = yaml.safe_load(yaml.serialize(previous)) if previous is not None else None
            with receipt.open('x', encoding='utf-8') as stream:
                os.chmod(receipt, 0o600)
                json.dump(record, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
        for config in configs:
            if config.path in staged:
                if config.path.read_bytes() != config.raw:
                    raise ParameterDefaultsError(f'저장 도중 설정 파일이 변경되었습니다: {config.path}')
                os.replace(staged[config.path], config.path)
                replaced.append(config)
        if receipt:
            record['status'] = 'committed'
            history_stage = _stage(receipt, (json.dumps(record, ensure_ascii=False, indent=2)+'\n').encode(), 0o600)
            os.replace(history_stage, receipt)
        return [str(config.path) for config in configs]
    except BaseException as exc:
        restore_errors = []
        for config in reversed(replaced):
            restore = None
            try:
                if config.path.read_bytes() != outputs[config.path]:
                    raise ParameterDefaultsError(
                        '저장 후 다른 변경이 있어 자동 복구로 덮어쓰지 않습니다')
                restore = _stage(config.path, config.raw, config.mode)
                os.replace(restore, config.path)
            except BaseException as restore_exc:
                restore_errors.append(f'{config.path}: {restore_exc}; 백업: {backups.get(config.path)}')
            finally:
                if restore is not None:
                    restore.unlink(missing_ok=True)
        if receipt is not None and not restore_errors:
            receipt.unlink(missing_ok=True)
        if restore_errors:
            raise ParameterDefaultsError('저장 실패 후 원본 복구도 실패했습니다: ' + '; '.join(restore_errors)) from exc
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise ParameterDefaultsError(f'기본값 저장 실패; 원본 파일을 유지/복구했습니다: {exc}') from exc
    finally:
        for path in staged.values():
            path.unlink(missing_ok=True)


def save_parameter_defaults(paths, changes):
    """Save accepted edits into bound source files; return canonical path strings.

    Nodes must match exactly one file. Every edit and every output is validated
    before any source changes. Changed sources get unique timestamped backups;
    if replacing a later source fails, earlier replacements are rolled back.
    """
    confirmed = _changes(changes, persistent=True)
    if not confirmed:
        return []
    configs, seen = [], set()
    for path in paths:
        config = _read(path)
        if config.path not in seen:
            configs.append(config)
            seen.add(config.path)
    grouped = {}
    for key, value in confirmed.items():
        matches = [config for config in configs if key[0] in config.sections]
        if len(matches) != 1:
            detail = ', '.join(str(config.path) for config in matches) or '없음'
            raise ParameterDefaultsError(f'노드 {key[0]}의 저장 대상은 정확히 하나여야 합니다: {detail}')
        grouped.setdefault(matches[0].path, {})[key] = value
    touched = [config for config in configs if config.path in grouped]
    outputs = {config.path: _render(config, grouped[config.path]) for config in touched}
    return _commit(touched, outputs, confirmed)


def restore_parameter_history(receipt_path):
    """Restore a saved transaction only if no later edits would be overwritten.

    Restoration itself is recorded as a new transaction; live ROS parameters
    remain unchanged until the operator applies them or restarts the nodes.
    """
    receipt = json.loads(Path(receipt_path).read_text(encoding='utf-8'))
    if receipt.get('version') != 1 or receipt.get('status') != 'committed':
        raise ParameterDefaultsError('완료된 파라미터 저장 이력만 복원할 수 있습니다.')
    configs, outputs = [], {}
    for item in receipt.get('files', []):
        config = _read(item['path'])
        backup = Path(item['backup']).read_bytes()
        if hashlib.sha256(config.raw).hexdigest() != item['after_sha256']:
            raise ParameterDefaultsError(f'이력 저장 후 다른 변경이 있어 복원을 중단합니다: {config.path}')
        if hashlib.sha256(backup).hexdigest() != item['before_sha256']:
            raise ParameterDefaultsError(f'백업 해시가 일치하지 않습니다: {item["backup"]}')
        yaml.compose(backup.decode('utf-8'))
        configs.append(config)
        outputs[config.path] = backup
    if not configs:
        raise ParameterDefaultsError('복원할 파일이 없습니다.')
    return _commit(configs, outputs)


def export_parameters(source, target, changes):
    """Compatibility export: preserve full source YAML and overlay accepted edits.

    Explicit exports may include runtime settings; only the defaults-save API
    enforces the operational-state exclusion policy.
    """
    config = _read(source)
    confirmed = _changes(changes, persistent=False)
    raw = _render(config, confirmed)
    target = Path(target).expanduser().resolve()
    if target == config.path:
        _commit([config], {config.path: raw})
        return
    mode = stat.S_IMODE(target.stat().st_mode) if target.exists() else config.mode
    staged = _stage(target, raw, mode)
    try:
        os.replace(staged, target)
    finally:
        staged.unlink(missing_ok=True)

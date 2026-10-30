"""Project DDS settings stay inside launched process trees and honor site setup."""

import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
HELPER = REPO / 'scripts/dds_env.bash'
PROFILE = PACKAGE / 'config/dds_udp.xml'


def run_shell(command, **overrides):
    environment = dict(os.environ)
    for key in ('SMART_VISION_DDS_TRANSPORT', 'SMART_VISION_DDS_PUBLICATION_MODE',
                'RMW_FASTRTPS_PUBLICATION_MODE', 'RMW_FASTRTPS_USE_QOS_FROM_XML', 'RMW_IMPLEMENTATION',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                '_SMART_VISION_DDS_OWN_PROFILE', 'ROS_LOCALHOST_ONLY'):
        environment.pop(key, None)
    environment.update(overrides)
    return subprocess.run(['bash', '-c', command, 'dds-test', str(HELPER), str(PROFILE)],
                          env=environment, text=True, capture_output=True)


def test_default_udp_applies_to_child_process_only():
    result = run_shell('''
      (source "$1" "$2"; bash -c 'printf "%s\\n%s\\n" "$FASTRTPS_DEFAULT_PROFILES_FILE" "$FASTDDS_DEFAULT_PROFILES_FILE"')
      printf 'parent=%s\\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
    ''')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [str(PROFILE), str(PROFILE), 'parent=unset']


def test_default_rollback_clears_only_helper_owned_values():
    result = run_shell('''
      source "$1" "$2"
      export FASTDDS_DEFAULT_PROFILES_FILE=/site/later-profile.xml
      export SMART_VISION_DDS_TRANSPORT=default
      smart_vision_dds_env
      printf '%s\\n%s\\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}" "$FASTDDS_DEFAULT_PROFILES_FILE"
    ''')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['unset', '/site/later-profile.xml']


def test_publication_mode_is_opt_in_and_blank_preserves_current_default():
    for requested in ({}, {'SMART_VISION_DDS_PUBLICATION_MODE': ''}):
        result = run_shell('''
          source "$1" "$2"
          printf '%s\\n' "${RMW_FASTRTPS_PUBLICATION_MODE-unset}"
        ''', **requested)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == 'unset'
        result = run_shell('''
          source "$1" "$2"
          printf '%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE"
        ''', RMW_FASTRTPS_PUBLICATION_MODE='AUTO', **requested)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == 'AUTO'


def test_supported_publication_modes_are_exported_only_to_child_processes():
    for mode in ('ASYNCHRONOUS', 'SYNCHRONOUS', 'AUTO'):
        result = run_shell('''
          (source "$1" "$2"; bash -c 'printf "%s\\n" "$RMW_FASTRTPS_PUBLICATION_MODE"')
          printf 'parent=%s\\n' "${RMW_FASTRTPS_PUBLICATION_MODE-unset}"
        ''', SMART_VISION_DDS_PUBLICATION_MODE=mode)
        assert result.returncode == 0, result.stderr
        assert result.stdout.splitlines() == [mode, 'parent=unset']


def test_explicit_caller_publication_mode_has_priority_including_blank():
    for mode in ('ASYNCHRONOUS', 'SYNCHRONOUS', 'AUTO', ''):
        result = run_shell('''
          source "$1" "$2"
          printf 'publication=%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE"
        ''', SMART_VISION_DDS_PUBLICATION_MODE='ASYNCHRONOUS',
            RMW_FASTRTPS_PUBLICATION_MODE=mode)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == 'publication='+mode


def test_publication_mode_applies_with_native_transport_and_site_xml():
    for overrides in ({'SMART_VISION_DDS_TRANSPORT': 'default'},
                      {'FASTRTPS_DEFAULT_PROFILES_FILE': '/site/custom.xml'}):
        result = run_shell('''
          source "$1" /does/not/exist.xml
          printf '%s|%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE" "${FASTRTPS_DEFAULT_PROFILES_FILE-unset}"
        ''', SMART_VISION_DDS_PUBLICATION_MODE='ASYNCHRONOUS', **overrides)
        assert result.returncode == 0, result.stderr
        expected = '/site/custom.xml' if 'FASTRTPS_DEFAULT_PROFILES_FILE' in overrides else 'unset'
        assert result.stdout.strip() == 'ASYNCHRONOUS|'+expected


def test_publication_override_leaves_other_middleware_untouched():
    for requested in ('ASYNCHRONOUS', 'invalid'):
        result = run_shell('''
          source "$1" /does/not/exist.xml
          printf '%s|%s\\n' "${RMW_FASTRTPS_PUBLICATION_MODE-unset}" "${FASTRTPS_DEFAULT_PROFILES_FILE-unset}"
        ''', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', SMART_VISION_DDS_PUBLICATION_MODE=requested)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == 'unset|unset'


def test_unsupported_publication_mode_fails_before_changing_any_setting():
    for mode in ('async', 'asynchronous', ' ', 'INVALID'):
        result = run_shell('''
          source "$1" "$2"
          status=$?
          printf '%s|%s\\n' "${RMW_FASTRTPS_PUBLICATION_MODE-unset}" "${FASTRTPS_DEFAULT_PROFILES_FILE-unset}"
          exit "$status"
        ''', SMART_VISION_DDS_PUBLICATION_MODE=mode)
        assert result.returncode == 2
        assert 'SMART_VISION_DDS_PUBLICATION_MODE must be' in result.stderr
        assert result.stdout.strip() == 'unset|unset'


def test_publication_override_preserves_xml_qos_policy_and_later_caller_replacement():
    result = run_shell('''
      source "$1" "$2"
      printf '%s|%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE" "$RMW_FASTRTPS_USE_QOS_FROM_XML"
      export RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS
      smart_vision_dds_env "$2"
      printf '%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE"
      export SMART_VISION_DDS_PUBLICATION_MODE=
      smart_vision_dds_env "$2"
      printf '%s\\n' "$RMW_FASTRTPS_PUBLICATION_MODE"
    ''', SMART_VISION_DDS_PUBLICATION_MODE='ASYNCHRONOUS', RMW_FASTRTPS_USE_QOS_FROM_XML='1')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['ASYNCHRONOUS|1', 'SYNCHRONOUS', 'SYNCHRONOUS']


def test_user_profiles_are_never_overwritten():
    for variable in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        result = run_shell('''
          source "$1" /does/not/exist.xml
          printf '%s|%s\\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" "${FASTDDS_DEFAULT_PROFILES_FILE:-}"
        ''', **{variable: '/site/custom.xml'})
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == ('/site/custom.xml|' if variable.startswith('FASTRTPS')
                                         else '|/site/custom.xml')


def test_other_middleware_and_explicit_native_mode_are_untouched():
    for override in ({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
                     {'SMART_VISION_DDS_TRANSPORT': 'default'}):
        result = run_shell('''
          source "$1" /does/not/exist.xml
          printf '%s|%s\\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" "${FASTDDS_DEFAULT_PROFILES_FILE:-}"
        ''', **override)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == '|'


def test_bad_mode_and_missing_requested_profile_fail_clearly():
    result = run_shell('source "$1" "$2"', SMART_VISION_DDS_TRANSPORT='invalid')
    assert result.returncode == 2
    assert 'must be udp or default' in result.stderr
    result = run_shell('source "$1" /does/not/exist.xml')
    assert result.returncode == 2
    assert 'UDP DDS profile is missing' in result.stderr


def test_profile_is_udp_only_without_host_specific_interface_rules():
    namespace = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(PROFILE).getroot()
    assert root.find('.//dds:transport_descriptor/dds:type', namespace).text == 'UDPv4'
    assert root.find('.//dds:useBuiltinTransports', namespace).text == 'false'
    assert not root.findall('.//dds:interfaceWhiteList', namespace)
    assert '127.0.0.1' not in PROFILE.read_text()


def test_localhost_scope_and_later_physical_web_scope_select_correct_profile():
    result = run_shell('''
      export ROS_LOCALHOST_ONLY=1
      source "$1" "$(dirname -- "$2")"
      printf '%s\\n' "$FASTRTPS_DEFAULT_PROFILES_FILE"
      export ROS_LOCALHOST_ONLY=0
      smart_vision_dds_env "$(dirname -- "$2")"
      printf '%s\\n' "$FASTRTPS_DEFAULT_PROFILES_FILE"
    ''')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [str(PROFILE.with_name('dds_udp_localhost.xml')), str(PROFILE)]
    namespace = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(PROFILE.with_name('dds_udp_localhost.xml')).getroot()
    addresses = root.findall('.//dds:interfaceWhiteList/dds:address', namespace)
    assert [address.text for address in addresses] == ['127.0.0.1']


def test_scope_refresh_preserves_a_later_site_profile():
    result = run_shell('''
      export ROS_LOCALHOST_ONLY=1
      source "$1" "$(dirname -- "$2")"
      export FASTDDS_DEFAULT_PROFILES_FILE=/site/new.xml ROS_LOCALHOST_ONLY=0
      smart_vision_dds_env "$(dirname -- "$2")"
      printf '%s\\n%s\\n' "$FASTRTPS_DEFAULT_PROFILES_FILE" "$FASTDDS_DEFAULT_PROFILES_FILE"
    ''')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [str(PROFILE.with_name('dds_udp_localhost.xml')), '/site/new.xml']


def test_deployments_ship_the_same_profile_and_helper():
    assert "glob('config/*.xml')" in (PACKAGE / 'setup.py').read_text()
    deb = (REPO / 'distribution/controller/build_deb.py').read_text()
    assert "project / 'scripts/dds_env.bash'" in deb
    assert "config/dds_udp.xml'" in deb
    assert "config/dds_udp_localhost.xml'" in deb
    assert 'source "$APP_DIR/dds_env.bash" "$APP_DIR"' in (
        REPO / 'distribution/controller/turtlebot3-controller').read_text()
    assert "temporary / 'tools/dds_env.bash'" in (
        REPO / 'distribution/lab/build_lab_bundle.py').read_text()

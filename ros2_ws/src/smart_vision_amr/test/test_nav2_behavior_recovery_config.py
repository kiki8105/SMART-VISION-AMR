"""Keep the physical Nav2 behavior server compatible with Humble's recovery trees."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
NAV2_SHARE = Path('/opt/ros/humble/share')


@pytest.mark.parametrize('tree_name', [
    'navigate_to_pose_w_replanning_and_recovery.xml',
    'navigate_through_poses_w_replanning_and_recovery.xml',
])
def test_physical_behavior_server_provides_default_tree_recoveries(tree_name):
    params = yaml.safe_load((PACKAGE / 'config/curve_tuning_waffle_pi.yaml').read_text())
    behavior = params['behavior_server']['ros__parameters']
    tree = ET.parse(NAV2_SHARE / 'nav2_bt_navigator/behavior_trees' / tree_name)
    exported = {
        entry.attrib['name']
        for entry in ET.parse(NAV2_SHARE / 'nav2_behaviors/behavior_plugin.xml').iter('class')
    }

    # These BT action tags require matching action servers from behavior_server.
    required_actions = {'Spin', 'Wait', 'BackUp'}
    used_actions = {entry.tag for entry in tree.iter() if entry.tag in required_actions}
    assert used_actions == required_actions
    for action in used_actions:
        alias = action.lower()
        plugin = f'nav2_behaviors/{action}'
        assert alias in behavior['behavior_plugins'], f'{action} action server is missing'
        assert behavior[alias]['plugin'] == plugin
        assert plugin in exported

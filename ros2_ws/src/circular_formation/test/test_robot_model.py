import os
import xml.dom.minidom as minidom

import pytest

from circular_formation.robot_model import namespace_burger_sdf

HERE = os.path.dirname(__file__)
STOCK = open(os.path.join(HERE, 'fixtures', 'burger_minimal.sdf'),
             newline='').read()


def test_topics_and_frames_namespaced():
    out = namespace_burger_sdf(STOCK, 3)
    for want in ('<topic>/robot_3/cmd_vel</topic>',
                 '<odom_topic>/robot_3/odom</odom_topic>',
                 '<frame_id>robot_3/odom</frame_id>',
                 '<child_frame_id>robot_3/base_footprint</child_frame_id>',
                 '<tf_topic>/robot_3/tf_wheel</tf_topic>',
                 '<topic>/robot_3/joint_states</topic>'):
        assert want in out
    for bare in ('<topic>cmd_vel<', '<odom_topic>odom<', '<frame_id>odom<',
                 '<tf_topic>/tf<', '<topic>joint_states<'):
        assert bare not in out


def test_sensors_stripped_by_default_and_xml_stays_valid():
    out = namespace_burger_sdf(STOCK, 0)
    assert '<sensor' not in out
    minidom.parseString(out.encode())


def test_sensors_kept_and_namespaced_on_request():
    out = namespace_burger_sdf(STOCK, 2, strip_sensors=False)
    assert out.count('<sensor') == 2
    assert '<topic>/robot_2/scan</topic>' in out
    assert '<topic>/robot_2/imu</topic>' in out
    assert '<gz_frame_id>robot_2/base_scan</gz_frame_id>' in out
    minidom.parseString(out.encode())


def test_distinct_robots_get_distinct_topics():
    a, b = namespace_burger_sdf(STOCK, 0), namespace_burger_sdf(STOCK, 1)
    assert '/robot_1/' not in a and '/robot_0/' not in b


def test_changed_stock_model_fails_loudly():
    broken = STOCK.replace('<odom_topic>odom</odom_topic>',
                           '<odom_topic> odom </odom_topic>')
    with pytest.raises(RuntimeError, match='odom_topic'):
        namespace_burger_sdf(broken, 0)


def test_handles_crlf_stock_files():
    crlf = STOCK.replace('\n', '\r\n')
    out = namespace_burger_sdf(crlf, 4)
    assert '<sensor' not in out and '/robot_4/cmd_vel' in out

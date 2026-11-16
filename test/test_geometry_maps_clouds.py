import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rough_terrain_navigation.geometry import split_pose, quaternion_matrix, multiply_quaternions
from rough_terrain_navigation.maps import generate_map
from rough_terrain_navigation.clouds import read_xyzi, make_cloud, flatten
from rough_terrain_navigation.command_gate import allowed


def test_full_pose_reconstructed_from_planar_and_residual():
    for yaw in [-3., -.4, 1.7]:
        qx = [math.sin(.3/2), 0, 0, math.cos(.3/2)]
        qy = [0, math.sin(-.6/2), 0, math.cos(-.6/2)]
        qz = [0, 0, math.sin(yaw/2), math.cos(yaw/2)]
        original = multiply_quaternions(qz, multiply_quaternions(qy, qx))
        p, q, residual_p, residual_q = split_pose([-7.2, -3.2, 2.4], original)
        np.testing.assert_allclose(p+quaternion_matrix(q)@residual_p, [-7.2, -3.2, 2.4])
        np.testing.assert_allclose(quaternion_matrix(q)@quaternion_matrix(residual_q),
                                   quaternion_matrix(original), atol=1e-12)
        assert p[2] == 0
    with pytest.raises(ValueError):
        split_pose([0,0,0], [0,0,0,0])


def test_empty_map_has_only_boundary_and_correct_origin(tmp_path):
    path = generate_map(tmp_path, width=4., height=3., resolution=.1, origin_x=-2., origin_y=-1.5, boundary=.2)
    config = yaml.safe_load(Path(path).read_text())
    assert config['origin'] == [-2., -1.5, 0.]
    data = (tmp_path/'empty.pgm').read_bytes().split(b'\n',3)[3]
    grid = np.frombuffer(data,dtype=np.uint8).reshape(30,40)
    assert (grid[:2] == 0).all() and (grid[:, :2] == 0).all()
    assert (grid[2:-2,2:-2] == 254).all()
    with pytest.raises(ValueError):
        generate_map(tmp_path, width=1.03)


def test_cloud_roundtrip_and_flatten_keep_metric_xy():
    cloud = np.array([[1,2,3,4],[-1,5,6,8]],dtype=np.float32)
    np.testing.assert_array_equal(read_xyzi(make_cloud(cloud, Header())),cloud)
    projected = flatten(cloud,[10,-3,4])
    np.testing.assert_allclose(projected[:,:3],[[11,-1,.1],[9,2,.1]])
    assert len(read_xyzi(make_cloud(np.empty((0,4)),Header()))) == 0


def test_big_endian_organized_padded_cloud_without_intensity():
    msg=PointCloud2(height=2,width=2,point_step=12,row_step=32,is_bigendian=True)
    msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
    a=np.array([[1,2,3],[4,5,6]],dtype='>f4').tobytes()
    msg.data=a+bytes(8)+a+bytes(8)
    output=read_xyzi(msg)
    np.testing.assert_array_equal(output[:,:3],[[1,2,3],[4,5,6],[1,2,3],[4,5,6]])
    assert (output[:,3] == 0).all()


def test_gate_rejects_stale_sensor_stamp_even_if_recently_received():
    assert allowed(10,20,[(9.9,19.8),(9.8,19.9)],1.)
    assert not allowed(10,20,[(9.9,17.)],1.)
    assert not allowed(10,20,[None],1.)
    assert not allowed(10,20,[(8.,19.8)],1.)
    assert not allowed(10,20,[(9.9,21.)],1.)


def test_patchwork_pairing_restores_input_subsecond_stamp():
    from types import SimpleNamespace
    from rough_terrain_navigation.ground_pipeline import GroundPipeline
    class Publisher:
        def __init__(self):
            self.messages = []
        def publish(self, msg):
            self.messages.append(msg)
    header = Header(frame_id='navigation/lidar_level')
    header.stamp.sec, header.stamp.nanosec = 3, 123456789
    points = np.array([[1., 2., -.8, 0.]],dtype=np.float32)
    fake = SimpleNamespace(fault=False, p={'fixed_frame':'navigation/odom','max_relative_height':2.5},
        inflight={'header':header,'points':points,'translation':np.array([10.,20.,3.]),'results':{}},
        ground_pub=Publisher(),non_ground_pub=Publisher(),clear_pub=Publisher(),
        obstacle_pub=Publisher(),status_pub=Publisher())
    broken_header=Header(frame_id='wrong_upstream_frame')
    broken_header.stamp.sec=3
    GroundPipeline.result(fake,'nonground',make_cloud(points,broken_header))
    assert fake.inflight is not None
    assert not fake.obstacle_pub.messages
    GroundPipeline.result(fake,'ground',make_cloud(np.empty((0,4)),broken_header))
    assert fake.inflight is None
    result=fake.obstacle_pub.messages[0]
    assert result.header.stamp.nanosec == 123456789
    assert result.header.frame_id == 'navigation/odom'
    np.testing.assert_allclose(read_xyzi(result)[0,:3],[11.,22.,.1])

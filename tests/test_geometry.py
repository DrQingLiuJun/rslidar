"""Validate full ENU extrinsics, the inverted sensor, and marker geographic labels."""
import math,re,unittest
import numpy as np
from std_msgs.msg import Header
from lidar_perception.geometry import calc_tf_args, sensor_pose, rotation_matrix, enu_to_lonlat, heading_in_map
from lidar_perception.visualization import DetectionVisualizer

class GeometryTests(unittest.TestCase):
    def test_legacy_planar_pose_is_compatible(self):
        R,t=sensor_pose(34.5865343,113.6796418,.9268,.7425,127.7422)
        np.testing.assert_allclose(R,rotation_matrix(yaw_deg=127.7422));args=list(map(float,calc_tf_args(34.5865343,113.6796418,.9268,.7425,127.7422)))
        np.testing.assert_allclose(args[:3],t);np.testing.assert_allclose(args[3:],[math.radians(127.7422),0,0])

    def test_three_dimensional_tf_matches_marker_pose(self):
        R,t=sensor_pose(34.58656254,113.6788296,.042132,.318996,39.712641,z=4.228958,roll_deg=179.498312,pitch_deg=-2.044854)
        args=list(map(float,calc_tf_args(34.58656254,113.6788296,.042132,.318996,39.712641,z=4.228958,roll_deg=179.498312,pitch_deg=-2.044854)))
        np.testing.assert_allclose(rotation_matrix(*np.degrees(args[3:][::-1])),R,atol=1e-12);np.testing.assert_allclose(args[:3],t);np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(R),1)

    def test_upside_down_sensor_has_upward_world_ground(self):
        R=rotation_matrix(180,0,40);np.testing.assert_allclose(R@np.array([0,0,4])+[0,0,4],[0,0,0],atol=1e-12);self.assertAlmostEqual(heading_in_map(math.pi/2,R),310)

    def test_geographic_labels_use_same_pose_as_rviz(self):
        class Publisher:
            def publish(self,msg): self.msg=msg
        class Tracker:
            def update(self,boxes,labels,scores): return boxes,labels,scores,[1]
        pub=Publisher();view=DetectionVisualizer(['Vehicle'],pub,Tracker(),34.58656254,113.6788296,.042132,.318996,39.712641,roll=179.498312,pitch=-2.044854,height=4.228958)
        box=np.array([[2.,3.,4.,4.,2.,1.,.2]]);header=Header(frame_id='rslidar_3');view.publish(box,np.array([1]),np.array([.9]),header);world=view.rotation@box[0,:3]+view.translation;lon,lat=enu_to_lonlat(*world[:2]);label=pub.msg.markers[1]
        self.assertIn(f'Lat: {lat:.7f}',label.text);self.assertIn(f'Lon: {lon:.7f}',label.text);self.assertEqual(pub.msg.markers[0].header.frame_id,'rslidar_3');self.assertAlmostEqual(pub.msg.markers[0].pose.position.x,2);self.assertIn(f'Ang: {heading_in_map(.2,view.rotation):.1f}',label.text)

    def test_invalid_extrinsics_fail_before_publication(self):
        with self.assertRaises(ValueError): sensor_pose(float('nan'),113,0,0,0)

if __name__=='__main__': unittest.main()

DIRECTORY=/home/jim/software/catkin_ws_research/workspace/src/catkin/human/motion-prediction/data
DATA_FILE=mocap-mlr/interpolated/mocap_data_1517838370.hdf5
python visualize_skeleton_tf.py -f ${DIRECTORY}/${DATA_FILE} -j

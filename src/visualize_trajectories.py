def load_data_from_file():
    import h5py
    with h5py.File('./data/trajectories.hdf5', 'r') as f:
        configurations = f["configurations"][:]
        points = f["points"][:]
        trajectories = []
        for i in range(len(configurations)):
            trajectories.append(f["trajectories_{:04d}".format(i)][:])
    return configurations, points, trajectories

if __name__ == '__main__':
    configurations, points, trajectories = load_data_from_file()
    print "configurations.shape : ", configurations.shape
    print "points.shape : ", points.shape
    for trajectory in trajectories:
        print trajectory.shape
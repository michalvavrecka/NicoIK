import argparse
import subprocess
import sys
import numpy as np
import plotly.express as px


def read_file(file_name):
    joint_angles, positions = [], []

    with open(file_name, 'r') as f:
        content = f.read().split('\n')[1:-1]

    if content:
        for pos in content:
            joints, duration, end_effector_position = pos.split()
            joint_angles.append(joints.split(','))
            positions.append(list(map(float, end_effector_position.split(','))))
    else:
        print("File empty")

    return joint_angles, positions


def get_positions(joints):
    positions = []

    for i, joint_angles in enumerate(joints):
        print('Calculating position {} out of {}'.format(i+1, len(joints)), end='\r')
        prog = ['python', 'solver.py', '-pp', '-j'] + joint_angles
        position = subprocess.check_output(prog).decode(sys.stdout.encoding).split('\n')[0].split(' ')
        positions.append(list(map(float, position)))

    return positions


def plot(data):
    fig = px.scatter_3d(data, x=0, y=1, z=2)

    fig.update_layout(title="Trajectory Points",
                      scene=dict(xaxis_title="X",
                                 yaxis_title="Y",
                                 zaxis_title="Z",
                                 xaxis=dict(nticks=9, range=[-0.2, 0.7]),
                                 yaxis=dict(nticks=12, range=[-0.6, 0.6]),
                                 zaxis=dict(nticks=7, range=[0, 0.7]),
                                 aspectmode='manual',
                                 aspectratio=dict(x=1, y=12/9, z=7/9)
                                 )
                      )

    hover_text = ['X: ' + str(x) + '<br>Y: ' + str(y) + '<br>Z: ' + str(z)
                  for x, y, z in np.round(data, decimals=3)]
    fig.update_traces(hovertemplate=hover_text)

    fig.write_html('Trajectory.html', auto_open=True)


def main():
    parser = argparse.ArgumentParser(description='Plot a 3D trajectory')
    parser.add_argument("trajectory_file", type=str, help="Path to the trajectory file")
    parser.add_argument("-j", "--joints", action="store_true", help="If set, calculates positions from joint angles in the given file")
    arg_dict = vars(parser.parse_args())

    joints, positions = read_file(arg_dict['trajectory_file'])

    if arg_dict["joints"]:
        positions = get_positions(joints)

    plot(positions)


if __name__ == "__main__":
    main()

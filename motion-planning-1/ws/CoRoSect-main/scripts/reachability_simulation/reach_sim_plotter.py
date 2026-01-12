import numpy as np
import json
import plotly.graph_objects as go
import plotly.io as pio

# test_type = 'Reachability'
test_type = 'Manipulability'
lengths = [0.0]
heights = [0.0]

for height in heights:
    for length in lengths:
        base_position = [0, 0, height]

        try:
            filename = './save/KR10r1420_' + test_type + '_Rail_Length-' + str(length) + '_Mounting-Height-' + str(height) + '.json'
            print(filename)
            with open(filename) as json_file:
                data = json.load(json_file)
            json_file.close()

            values = np.array(data['scores'])
            print(values.shape)
            x = data['x']
            y = data['y']
            z = data['z']

            if test_type == 'Reachability':
                if height <= 0.0:  # only apply visual correction if not mounted
                    for i in range(len(x)):
                        for j in range(len(y)):
                            for k in range(len(z)):
                                point = np.array([x[i], y[j], z[k]])
                                if np.linalg.norm(base_position - point) < 0.820 and point[
                                    2] > 0.0:  # Natural Reach of KUKA iiwa R820
                                    values[i, j, k] = 1.0

            X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

            fig = go.Figure(data=go.Volume(
                x=X.flatten(), y=Y.flatten(), z=Z.flatten(),
                value=values.flatten(),
                isomin=0.0,
                isomax=1.0,
                opacity=0.1,
                surface_count=50,
                colorscale='rdylgn'
            ))

            name = 'Prototype ' + test_type + ' Rail Length-' + str(length) #+ ' Mounting Height-' + str(height)
            # Default parameters which are used when `layout.scene.camera` is not provided
            camera = dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=1.25, y=1.25, z=1.25)
            )

            fig.update_layout(scene_camera=camera,
                              width=1200,
                              height=800,
                              scene=dict(
                                  xaxis=dict(range=[-2, 2], ),
                                  yaxis=dict(range=[-2, 2], ),
                                  zaxis=dict(range=[0, 4], ), ),
                              margin=dict(l=20, r=20, t=20, b=20),
                              title=dict(text=name,
                                         font=dict(size=40),
                                         automargin=True,
                                         yref='paper'))

            fig.write_html(
                './plots/KR10r1420_' + test_type + '_Rail_Length-' + str(length) + '_Mounting-Height-' + str(height) + '_2.html',
                auto_open=False)
            # pio.write_image(fig,
            #                 './plots/' + test_type + '_Rail_Length-' + str(length) + '_Mounting-Height-' + str(height) + '_2.svg',
            #                 width=1200,
            #                 height=800,
            #                 scale=1)
        except FileNotFoundError as e:
            print('File does not exist')

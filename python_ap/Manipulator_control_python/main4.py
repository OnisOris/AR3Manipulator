import plotly.graph_objects as go

fig = go.Figure(data=go.Cone(
    x=[0, 0],
    y=[0, 0],
    z=[0, 0],
    # u=[1, 2],
    # v=[0, 2],
    # w=[0, 2],
    sizemode="absolute",
    sizeref=2,
    anchor="tip"))

fig.update_layout(
      scene=dict(domain_x=[0, 1],
                 camera_eye=dict(x=-1.57, y=1.36, z=0.58)))

fig.show()
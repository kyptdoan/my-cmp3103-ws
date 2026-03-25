import json, math

with open('/workspaces/my-cmp3103-ws/src/cube_pos.json') as f:
    cube = json.load(f)

with open('/workspaces/my-cmp3103-ws/src/patch_pos.json') as f:
    patch = json.load(f)

# vector from patch to cube
dx = cube['x'] - patch['x']
dy = cube['y'] - patch['y']
dist = math.hypot(dx, dy)

# normalise
nx = dx / dist
ny = dy / dist

# 5cm behind the cube (opposite direction)
standoff = 0.3
pre_push = {
    'x': round(cube['x'] + nx * standoff, 4),
    'y': round(cube['y'] + ny * standoff, 4),
    'z': round(math.sin(math.atan2(-ny, -nx) / 2), 4),
    'w': round(math.cos(math.atan2(-ny, -nx) / 2), 4),
}

print(pre_push)

print(f'patch: {patch}')
print(f'cube: {cube}')
print(f'nx={nx:.3f} ny={ny:.3f}')
print(f'pre_push: {pre_push}')

d_patch_to_cube = math.hypot(cube['x']-patch['x'], cube['y']-patch['y'])
d_patch_to_prepush = math.hypot(pre_push['x']-patch['x'], pre_push['y']-patch['y'])
print(f'patch→cube: {d_patch_to_cube:.3f}m')
print(f'patch→prepush: {d_patch_to_prepush:.3f}m')

with open('/workspaces/my-cmp3103-ws/src/pre_pushing_pose.json', 'w') as f:
    json.dump(pre_push, f)
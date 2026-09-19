# sura_actions

Servidores ROS 2 para comportamientos autónomos AUV y USV. El launch selecciona
la implementación según `robot_family`; `sura_control_manager_node` se ejecuta
para ambas familias.

| `robot_family` | Servidores |
| --- | --- |
| `underwater` | Surface, GoToDepth, GoToPose AUV y FollowPath AUV |
| `surface` | GoToPose USV y FollowPath USV |

Los ejemplos usan `cirtesub` para el AUV y `blueboat` para el USV. Ejecuta
siempre esto en cada terminal nueva:

```bash
cd ~/sura_blueboat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Lanzamiento

El bring-up obtiene la familia del atributo `family` del xacro:

```bash
ros2 launch sura_bringup sura_bringup.launch.py robot_namespace:=blueboat
```

Para lanzar solamente las acciones:

```bash
# USV
ros2 launch sura_actions actions.launch.py robot_namespace:=blueboat robot_family:=surface

# AUV
ros2 launch sura_actions actions.launch.py robot_namespace:=cirtesub robot_family:=underwater
```

Los servidores lifecycle arrancan en `unconfigured`. Consulta su estado con:

```bash
ros2 lifecycle nodes
ros2 lifecycle get /go_to_pose_usv_lifecycle_action_node
ros2 lifecycle get /path_follower_usv_lifecycle_node
ros2 lifecycle get /go_to_pose_lifecycle_action_node
ros2 lifecycle get /path_manager_lifecycle_node
```

`configure` crea las acciones, servicios y markers y deja el nodo en
`inactive`. La edición se realiza en `inactive`; los goals se aceptan en
`active`. Para volver a editar utiliza `deactivate`.

## USV: GoToPose

Configurar y activar:

```bash
ros2 lifecycle set /go_to_pose_usv_lifecycle_action_node configure
ros2 lifecycle set /go_to_pose_usv_lifecycle_action_node activate
```

Enviar una pose XY/yaw explícita (`theta` está en radianes):

```bash
ros2 action send_goal /blueboat/actions/go_to_pose \
  sura_actions/action/GoToPoseUsv \
  "{target_pose: {x: 5.0, y: 2.0, theta: 1.57}, use_planned_pose: false}" \
  --feedback
```

Para planificar con RViz, configura el nodo y déjalo en `inactive`. Usa
`Fixed Frame: world_ned` y añade:

- `Pose`: `/blueboat/actions/go_to_pose/target_pose`
- `InteractiveMarkers`: `/blueboat/actions/go_to_pose/interactive_marker/update`

Mueve `go_to_pose_goal`, activa el nodo y ejecuta la pose planificada:

```bash
ros2 lifecycle set /go_to_pose_usv_lifecycle_action_node activate
ros2 action send_goal /blueboat/actions/go_to_pose \
  sura_actions/action/GoToPoseUsv "{use_planned_pose: true}" --feedback
```

Los controles X/Y del marker giran con su yaw. El USV solo ordena
`linear.x >= 0` y `angular.z`; no usa movimiento lateral, Z, roll ni pitch.
Los campos numéricos a cero toman los valores de `config/go_to_pose_usv.yaml`.

## USV: crear, guardar, cargar y ejecutar un path

Configura el editor y déjalo en `inactive`:

```bash
ros2 lifecycle set /path_follower_usv_lifecycle_node configure
```

Si ya estaba activo:

```bash
ros2 lifecycle set /path_follower_usv_lifecycle_node deactivate
```

Añadir waypoints:

```bash
ros2 service call /blueboat/path_manager/add_waypoint \
  sura_actions/srv/AddWaypointUsv "{x: 2.0, y: 0.0, yaw: 0.0}"

ros2 service call /blueboat/path_manager/add_waypoint \
  sura_actions/srv/AddWaypointUsv "{x: 5.0, y: 2.0, yaw: 1.57}"
```

Eliminar el último waypoint o limpiar el path completo:

```bash
ros2 service call /blueboat/path_manager/remove_last_waypoint std_srvs/srv/Trigger "{}"
ros2 service call /blueboat/path_manager/clear_path std_srvs/srv/Trigger "{}"
```

Guardar el path actual:

```bash
ros2 service call /blueboat/path_manager/save_path \
  sura_actions/srv/PathFile \
  "{path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/usv/path_blueboat_port.xml'}"
```

Cargar el XML para verlo o editarlo:

```bash
ros2 service call /blueboat/path_manager/load_path \
  sura_actions/srv/PathFile \
  "{path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/usv/path_blueboat_port.xml'}"
```

En RViz usa:

- `Path`: `/blueboat/path_manager/path`
- `MarkerArray`: `/blueboat/path_manager/markers`
- `InteractiveMarkers`: `/blueboat/path_manager/interactive_markers/update`

Los markers solo permiten XY/yaw y sus controles X/Y giran con el yaw de cada
waypoint. Para ejecutar el XML, activa el servidor e indica siempre
`path_file`; cargarlo previamente con el servicio no sustituye ese campo:

```bash
ros2 lifecycle set /path_follower_usv_lifecycle_node activate

ros2 action send_goal /blueboat/actions/follow_path \
  sura_actions/action/FollowPathUsv \
  "{use_saved_path: true, path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/usv/path_blueboat_port.xml'}" \
  --feedback
```

También puede enviarse un path directamente:

```bash
ros2 action send_goal /blueboat/actions/follow_path \
  sura_actions/action/FollowPathUsv \
  "{path: [{x: 2.0, y: 0.0, theta: 0.0}, {x: 5.0, y: 2.0, theta: 1.57}], use_saved_path: false}" \
  --feedback
```

El yaw de los waypoints intermedios se ignora. En el último waypoint el robot
alcanza XY y después alinea el yaw final. Los valores a cero usan
`config/path_follower_usv.yaml`.

## AUV: GoToPose

Configurar y activar:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node configure
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
```

Enviar una pose explícita:

```bash
ros2 action send_goal /cirtesub/actions/go_to_pose \
  sura_actions/action/GoToPose \
  "{target_pose: {header: {frame_id: 'world_ned'}, pose: {position: {x: 2.0, y: 1.0, z: -1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, target_yaw: 1.57, holonomic: true}" \
  --feedback
```

Para usar el marker, configura el nodo y déjalo en `inactive`. En RViz añade:

- `Pose`: `/cirtesub/actions/go_to_pose/target_pose`
- `InteractiveMarkers`: `/cirtesub/actions/go_to_pose/interactive_marker/update`

Después de moverlo:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
ros2 action send_goal /cirtesub/actions/go_to_pose \
  sura_actions/action/GoToPose "{}" --feedback
```

`holonomic: true` permite velocidad lateral; `false` usa navegación no
holonómica. Los valores omitidos se toman de `config/go_to_pose.yaml`.

## AUV: crear, guardar, cargar y ejecutar un path

Configurar el editor:

```bash
ros2 lifecycle set /path_manager_lifecycle_node configure
```

Añadir waypoints XYZ/yaw:

```bash
ros2 service call /cirtesub/path_manager/add_waypoint \
  sura_actions/srv/AddWaypoint "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0}"

ros2 service call /cirtesub/path_manager/add_waypoint \
  sura_actions/srv/AddWaypoint "{x: 3.0, y: 1.0, z: -1.5, yaw: 1.57}"
```

Eliminar, limpiar, guardar y cargar:

```bash
ros2 service call /cirtesub/path_manager/remove_last_waypoint std_srvs/srv/Trigger "{}"
ros2 service call /cirtesub/path_manager/clear_path std_srvs/srv/Trigger "{}"

ros2 service call /cirtesub/path_manager/save_path \
  sura_actions/srv/PathFile \
  "{path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/auv_path.xml'}"

ros2 service call /cirtesub/path_manager/load_path \
  sura_actions/srv/PathFile \
  "{path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/auv_path.xml'}"
```

Estos servicios requieren el nodo `inactive`. En RViz usa:

- `Path`: `/cirtesub/path_manager/path`
- `MarkerArray`: `/cirtesub/path_manager/markers`
- `InteractiveMarkers`: `/cirtesub/path_manager/interactive_markers/update`

Ejecutar el fichero guardado:

```bash
ros2 lifecycle set /path_manager_lifecycle_node activate

ros2 action send_goal /cirtesub/actions/follow_path \
  sura_actions/action/FollowPath \
  "{use_saved_path: true, path_file: '/home/salva/sura_blueboat_ws/src/sura_actions/config/paths/auv_path.xml', holonomic: true}" \
  --feedback
```

Enviar un path directamente:

```bash
ros2 action send_goal /cirtesub/actions/follow_path \
  sura_actions/action/FollowPath \
  "{use_saved_path: false, holonomic: false, path: {header: {frame_id: 'world_ned'}, poses: [{header: {frame_id: 'world_ned'}, pose: {position: {x: 0.0, y: 0.0, z: -1.0}, orientation: {w: 1.0}}}, {header: {frame_id: 'world_ned'}, pose: {position: {x: 3.0, y: 1.0, z: -1.5}, orientation: {w: 1.0}}}]}}" \
  --feedback
```

Los valores omitidos se toman de `config/path_manager.yaml`.

## Otras acciones AUV

Surface no es lifecycle:

```bash
ros2 action send_goal /cirtesub/actions/surface \
  sura_actions/action/Surface \
  "{target_depth: 0.0, depth_tolerance: 0.1, timeout: 30.0, surface_force_z: 0.4}" \
  --feedback
```

GoToDepth sí es lifecycle:

```bash
ros2 lifecycle set /go_to_depth_lifecycle_action_node configure
ros2 lifecycle set /go_to_depth_lifecycle_action_node activate
ros2 action send_goal /cirtesub/actions/go_to_depth \
  sura_actions/action/GoToDepth \
  "{target_depth: 2.0, depth_tolerance: 0.1, timeout: 30.0}" \
  --feedback
```

## Diagnóstico

Comprobar interfaces, servicios y acciones:

```bash
ros2 interface show sura_actions/action/GoToPoseUsv
ros2 interface show sura_actions/action/FollowPathUsv
ros2 interface show sura_actions/action/GoToPose
ros2 interface show sura_actions/action/FollowPath
ros2 interface show sura_actions/srv/PathFile
ros2 action list -t
ros2 service list -t | grep path_manager
```

Si aparece `The passed action/service type is invalid`, vuelve a cargar el
workspace en esa misma terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/sura_blueboat_ws/install/setup.bash
```

## Compilación

```bash
cd ~/sura_blueboat_ws
colcon build --packages-select sura_msgs sura_actions --symlink-install
source install/setup.bash
```

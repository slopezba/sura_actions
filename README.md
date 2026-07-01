# sura_actions

Paquete ROS 2 con acciones y servicios para comportamientos autonomos de SURA/CIRTESUB: salir a superficie, ir a una pose, planear un path con waypoints y ejecutar un path.

Los ejemplos usan el namespace `cirtesub`. Si lanzas con otro namespace, cambia `/cirtesub/...` por el que corresponda.

## Lanzar el paquete

Desde una terminal:

```bash
cd /home/cirtesu/cirtesub_ws
source install/setup.bash
ros2 launch sura_actions actions.launch.py robot_namespace:=cirtesub
```

Nodos principales:

```text
sura_control_manager_node
surface_action_node
go_to_pose_lifecycle_action_node
path_manager_lifecycle_node
```

Los nodos lifecycle no quedan activos al lanzar. Primero hay que configurarlos.

## Lifecycle

Ver nodos lifecycle:

```bash
ros2 lifecycle nodes
```

Ver estado:

```bash
ros2 lifecycle get /go_to_pose_lifecycle_action_node
ros2 lifecycle get /path_manager_lifecycle_node
```

Configurar un nodo lifecycle. Esto lo deja en `inactive`:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node configure
ros2 lifecycle set /path_manager_lifecycle_node configure
```

Activar un nodo lifecycle. En `active` acepta goals:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
ros2 lifecycle set /path_manager_lifecycle_node activate
```

Volver a modo planificacion/edicion:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node deactivate
ros2 lifecycle set /path_manager_lifecycle_node deactivate
```

Regla de uso:

- `go_to_pose_lifecycle_action_node`: en `inactive` puedes mover el marker de goal; en `active` acepta goals.
- `path_manager_lifecycle_node`: en `inactive` puedes planear/editar waypoints; en `active` acepta `FollowPath`.

## Servicios

### Cambiar modo de control

Servicio:

```text
/cirtesub/control_manager/set_mode
```

Tipo:

```text
sura_actions/srv/SetControlMode
```

Modos disponibles:

```text
MANUAL=0
FOLLOW_PATH=1
SURFACE=2
HOLD_POSITION=3
EMERGENCY_STOP=4
BODY_VELOCITY=5
```

Ejemplos:

```bash
ros2 service call /cirtesub/control_manager/set_mode sura_actions/srv/SetControlMode "{mode: 0, reason: 'manual test'}"
```

```bash
ros2 service call /cirtesub/control_manager/set_mode sura_actions/srv/SetControlMode "{mode: 5, reason: 'body velocity test'}"
```

Normalmente no hace falta llamar este servicio a mano para las acciones, porque los action servers lo solicitan internamente.

### Path manager

Estos servicios existen bajo `/cirtesub/path_manager/...`.

Anadir waypoint:

```bash
ros2 service call /cirtesub/path_manager/add_waypoint sura_actions/srv/AddWaypoint "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0}"
```

Borrar ultimo waypoint:

```bash
ros2 service call /cirtesub/path_manager/remove_last_waypoint std_srvs/srv/Trigger "{}"
```

Limpiar path:

```bash
ros2 service call /cirtesub/path_manager/clear_path std_srvs/srv/Trigger "{}"
```

Guardar path en XML:

```bash
ros2 service call /cirtesub/path_manager/save_path sura_actions/srv/PathFile "{path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}"
```

Cargar path desde XML:

```bash
ros2 service call /cirtesub/path_manager/load_path sura_actions/srv/PathFile "{path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}"
```

Para usar estos servicios de edicion, `path_manager_lifecycle_node` debe estar en `inactive`.

## RViz

### Ver y editar paths

Configura el path manager:

```bash
ros2 lifecycle set /path_manager_lifecycle_node configure
```

En RViz:

- `Fixed Frame`: `world_ned`
- Add -> `Path`: `/cirtesub/path_manager/path`
- Add -> `MarkerArray`: `/cirtesub/path_manager/markers`
- Add -> `InteractiveMarkers`
  - `Update Topic`: `/cirtesub/path_manager/interactive_markers/update`

### Mover el goal de GoToPose

Configura el nodo:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node configure
```

En RViz:

- Add -> `InteractiveMarkers`
  - `Update Topic`: `/cirtesub/actions/go_to_pose/interactive_marker/update`
- Para ver la pose publicada:
  - Add -> `Pose`: `/cirtesub/actions/go_to_pose/target_pose`

Mueve el marker en X/Y/Z/yaw. Luego activa el nodo y manda un goal sin pose explicita para usar esa pose.

## Actions

Listar actions:

```bash
ros2 action list
```

Ver la interfaz de una action:

```bash
ros2 interface show sura_actions/action/GoToPose
ros2 interface show sura_actions/action/FollowPath
ros2 interface show sura_actions/action/Surface
```

### Surface

Action:

```text
/cirtesub/actions/surface
```

Mandar goal:

```bash
ros2 action send_goal /cirtesub/actions/surface sura_actions/action/Surface "{target_depth: 0.0, depth_tolerance: 0.1, timeout: 30.0, surface_force_z: 0.4}" --feedback
```

### GoToPose con marker

Primero planea la pose con el marker en `inactive`:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node configure
```

Mueve el marker en RViz. Luego activa:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
```

Mandar goal usando la pose actual del marker:

```bash
ros2 action send_goal /cirtesub/actions/go_to_pose sura_actions/action/GoToPose "{}" --feedback
```

Los limites y tolerancias que no mandes se toman de `config/go_to_pose.yaml`, en la seccion `defaults`.

Para sobrescribir solo un campo:

```bash
ros2 action send_goal /cirtesub/actions/go_to_pose sura_actions/action/GoToPose "{holonomic: true}" --feedback
```

Con `holonomic: true` el robot puede usar velocidad lateral. Con `holonomic: false` se comporta como no holonomico.

Con `hold_after_reaching: true` (valor por defecto en `config/go_to_pose.yaml`), al entrar
en tolerancia la accion publica el estado `holding_target` y mantiene activo el mismo lazo
de correccion de `GoToPose`. No activa `position_hold`. La accion termina y el nodo vuelve
a `inactive` cuando se cancela o se desactiva. Si se cancela despues de haber alcanzado la
pose, el resultado conserva `success: true` y comunica que la pose se mantuvo correctamente
hasta la cancelacion. Una cancelacion anterior a alcanzar la pose devuelve `success: false`.

Si quieres el comportamiento anterior, configura `hold_after_reaching: false`: la accion
termina con exito al entrar en tolerancia, publica velocidad cero y vuelve automaticamente
a `inactive`.

### GoToPose con pose explicita

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
```

```bash
ros2 action send_goal /cirtesub/actions/go_to_pose sura_actions/action/GoToPose "{
  target_pose: {
    header: {frame_id: 'world_ned'},
    pose: {
      position: {x: 1.0, y: 0.0, z: -1.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  target_yaw: 0.0,
  holonomic: true,
  max_forward_speed: 0.3,
  max_vertical_speed: 0.3,
  max_yaw_rate: 0.5,
  position_tolerance: 0.2,
  yaw_tolerance: 0.2,
  slowdown_distance: 0.5,
  timeout: 60.0
}" --feedback
```

### FollowPath desde XML

El path manager debe estar activo para aceptar goals:

```bash
ros2 lifecycle set /path_manager_lifecycle_node activate
```

Ejecutar un path guardado:

```bash
ros2 action send_goal /cirtesub/actions/follow_path sura_actions/action/FollowPath "{use_saved_path: true, path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}" --feedback
```

Los limites y tolerancias que no mandes se toman de `config/path_manager.yaml`, en la seccion `defaults`.

Para sobrescribir solo un campo:

```bash
ros2 action send_goal /cirtesub/actions/follow_path sura_actions/action/FollowPath "{use_saved_path: true, path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml', holonomic: true}" --feedback
```

Con `hold_after_reaching: true` en `config/path_manager.yaml`, al completar el ultimo
waypoint la accion mantiene esa pose usando el mismo control de `FollowPath`, sin activar
`position_hold`. El feedback permanece con `progress: 1.0` y
`state: holding_final_waypoint` hasta cancelar o desactivar.
Si se cancela despues de completar el path, el resultado conserva `success: true`; una
cancelacion anterior devuelve `success: false`.

Con `hold_after_reaching: false`, `FollowPath` termina con exito inmediatamente despues
de alcanzar el ultimo waypoint.

### FollowPath mandando el path en el goal

```bash
ros2 lifecycle set /path_manager_lifecycle_node activate
```

```bash
ros2 action send_goal /cirtesub/actions/follow_path sura_actions/action/FollowPath "{
  use_saved_path: false,
  holonomic: false,
  goal_tolerance: 0.2,
  yaw_tolerance: 0.2,
  max_forward_speed: 0.3,
  max_yaw_rate: 0.5,
  timeout: 120.0,
  path: {
    header: {frame_id: 'world_ned'},
    poses: [
      {
        header: {frame_id: 'world_ned'},
        pose: {
          position: {x: 0.0, y: 0.0, z: -1.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
      },
      {
        header: {frame_id: 'world_ned'},
        pose: {
          position: {x: 2.0, y: 0.0, z: -1.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
      }
    ]
  }
}" --feedback
```

## Topics utiles

```text
/cirtesub/navigator/navigation
/cirtesub/controller/body_velocity/setpoint
/cirtesub/controller/depth_hold/set_point
/cirtesub/controller/depth_hold/feedforward
/cirtesub/actions/go_to_pose/target_pose
/cirtesub/path_manager/path
/cirtesub/path_manager/markers
```

## Resumen rapido

Planear pose con marker:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node configure
```

Ejecutar esa pose:

```bash
ros2 lifecycle set /go_to_pose_lifecycle_action_node activate
ros2 action send_goal /cirtesub/actions/go_to_pose sura_actions/action/GoToPose "{}" --feedback
```

Planear path:

```bash
ros2 lifecycle set /path_manager_lifecycle_node configure
ros2 service call /cirtesub/path_manager/add_waypoint sura_actions/srv/AddWaypoint "{x: 0.0, y: 0.0, z: -1.0, yaw: 0.0}"
ros2 service call /cirtesub/path_manager/add_waypoint sura_actions/srv/AddWaypoint "{x: 2.0, y: 0.0, z: -1.0, yaw: 0.0}"
```

Cargar path desde XML en modo configurado/inactive:

```bash
ros2 service call /cirtesub/path_manager/load_path sura_actions/srv/PathFile "{path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}"
```

Guardar y ejecutar path:

```bash
ros2 service call /cirtesub/path_manager/save_path sura_actions/srv/PathFile "{path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}"
ros2 lifecycle set /path_manager_lifecycle_node activate
ros2 action send_goal /cirtesub/actions/follow_path sura_actions/action/FollowPath "{use_saved_path: true, path_file: '/home/cirtesu/cirtesub_ws/src/sura_actions/config/paths/netinspection_path.xml'}" --feedback
```

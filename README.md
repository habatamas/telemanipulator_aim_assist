# Telemanipulator Aim Assist

ROS alapú alkalmazás, amellyel az OpenMANIPULATOR egyszerre szabadon és precíz pozícionálással mozgatható.

## Dependenciák

A csomag ROS Melodic platformra lett fejlesztve, megfelelő működéséhez a hivatalos OpenMANIPULATOR csomagokat is szükséges telepíteni az alábbi módon:

```
sudo apt-get install ros-melodic-open-manipulator-gazebo
sudo apt-get install ros-melodic-open-manipulator-controller
sudo apt-get install ros-melodic-open-manipulator-control-gui
sudo apt-get install ros-melodic-open-manipulator-description
```

További részletek a ROBOTIS hivatalos [weboldalán](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/).

## Telepítés

Először lépjünk be a catkin workspace-ünk forrásmappájába:
```
cd ~/catkin_ws/src
```

Klónozzuk a repót:
```
git clone https://github.com/habatamas/telemanipulator_aim_assist.git
```

Fordítsuk le a catkin workspace-t:
```
cd ~/catkin_ws
catkin_make
```

Töltsük be a környezetet:
```
source devel/setup.bash
```

## Szimulált környezet tesztelése
A környezet a következő paranccsal indítható el
```
roslaunch telemanipulator_aim_assist telemanipulator_aim_assist_rviz.launch
```
<details>
<summary>Lépésenként is végihaladhatunk</summary>

Először indítsuk el a robotkar és a környezet gazebo szimulációját:

```
roslaunch telemanipulator_aim_assist open_manipulator_gazebo.launch
```

Ezután indítsuk el a robotkar kinematikai vezérlőjét:

```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```

Majd az rviz-t:

```
roslaunch telemanipulator_aim_assist telemanipulator_aim_assist_rviz.launch
```

A teszteléshez indítsuk el a vezérlő GUI-t:

```
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```

Nyomjuk meg a __Timer Start__ gombot, majd a __Task Space__ fület megnyitva tetszőleges TCP koordináták adhatók meg. A __Send__ gomb megnyomásával a robot a beállítótt pozícióba mozgatható.

A koordinátor node indításához az alábbi szkriptet kell futtatni:

```
rosrun telemanipulator_aim_assist coordinator.py
```

A telemanipulátor kézi vezérléshez a következő szkriptet kell futtatni:

```
rosrun telemanipulator_aim_assist controller_keyboard.py
```
A telemanipulátor interaktív markeres vezérléshez a következő szkriptet kell futtatni:

```
rosrun telemanipulator_aim_assist controller_interactive_marker.py
```
</details>

## TODO

- végleges szimulált környezet elkészítése
- rviz integráció
- megfogás szimulálása
- saját vezérlő készítése
- mozgatási korlátok és aim assist implementálása
- package.xml testreszabása

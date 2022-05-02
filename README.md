# Telemanipulator Aim Assist

ROS alapú alkalmazás, amellyel az OpenMANIPULATOR egyszerre szabadon és precíz pozícionálással mozgatható.

## Dependenciák

A csomag ROS Melodic platformra lett fejlesztve, megfelelő működéséhez a hivatalos OpenMANIPULATOR csomagokat is szükséges telepíteni az alábbi módon:

```
sudo apt-get install ros-melodic-open-manipulator-gazebo
sudo apt-get install ros-melodic-open-manipulator-controller
sudo apt-get install ros-melodic-open-manipulator-control-gui
```

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

Először indítsuk el a robotkar és a környezet gazebo szimulációját:

```
roslaunch telemanipulator_aim_assist open_manipulator_gazebo.launch
```

Ezután indítsuk el a robotkar kinematikai vezérlőjét:

```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```

A teszteléshez indítsuk el a vezérlő GUI-t:

```
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```

Nyomjuk meg a __Timer Start__ gombot, majd a __Task Space__ fület megnyitva tetszőleges TCP koordináták adhatók meg. A __Send__ gomb megnyomásával a robot a beállítótt pozícióba mozgatható.

A telemanipulátor kézi vezérléshez a következő szkriptet kell futtatni:

```
rosrun telemanipulator_aim_assist controller_keyboard.py
```

## TODO

- végleges szimulált környezet elkészítése
- rviz integráció
- megfogás szimulálása
- saját vezérlő készítése
- mozgatási korlátok és aim assist implementálása
- package.xml testreszabása
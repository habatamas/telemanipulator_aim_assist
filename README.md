# Telemanipulator Aim Assist

ROS alapú alkalmazás, amely célja, hogy az OpenMANIPULATOR robotkart szabad mozgatás mellett precízen lehessen pozicionálni a megadott célpontok fölé.

Az alkalmazás valós és szimulált környezetben is implementálva lett, az alábbi demo videón megtekinthető a működés.
[![Project demo video](https://img.youtube.com/vi/XOpsILf6k4w/0.jpg)](https://youtu.be/XOpsILf6k4w)

# Tartalomjegyzék
1. [Projekt felépítése](#Projekt-felépítése)
2. [Alkalmazás használata](#Alkalmazás-használata)
2.1. [Dependenciák és telepítés](#Dependenciák-és-telepítés)  
2.2. [Szimulált környezet](#Szimulált-környezet)  
2.2.1. [Irányítás koordináták megadásával](#Irányítás-koordináták-megadásával)  
2.2.2. [Irányítás interaktív markerrel](#Irányítás-interaktív-markerrel)  
2.3. [Irányítás MOGI haptikus eszközzel valós környezetben](#Irányítás-MOGI-haptikus-eszközzel-valós-környezetben)  

# Projekt felépítése 


# Alkalmazás használata

## Dependenciák és telepítés

A csomag ROS Melodic platformra lett fejlesztve, megfelelő működéséhez a hivatalos OpenMANIPULATOR csomagokat is szükséges telepíteni az alábbi módon:

```
sudo apt-get install ros-melodic-open-manipulator-gazebo
sudo apt-get install ros-melodic-open-manipulator-controller
sudo apt-get install ros-melodic-open-manipulator-control-gui
sudo apt-get install ros-melodic-open-manipulator-description
```

További részletek a ROBOTIS hivatalos [weboldalán](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/).

**Telepítés**

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

## Szimulált környezet 
### Irányítás koordináták megadásával
asdadada
### Irányítás interaktív markerrel
A környezet a következő paranccsal indítható el
```
roslaunch telemanipulator_aim_assist telemanipulator_aim_assist_rviz.launch
```
<details>
<summary>A folyamat lépésekre bontva</summary>

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

### Irányítás MOGI haptikus eszközzel valós környezetben


<!---## TODO

- végleges szimulált környezet elkészítése
- rviz integráció
- megfogás szimulálása
- saját vezérlő készítése
- mozgatási korlátok és aim assist implementálása
- package.xml testreszabása 
-->

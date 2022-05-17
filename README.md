[//]: # (Image References)

[image1]: ./assets/constraint_space.png "Kényszerkörnyezet"
[image2]: ./assets/gazebo_world_screencap.png "Gazebo world"
[image3]: ./assets/rviz_screencap.png "RViz ablak"

# Telemanipulator Aim Assist

ROS alapú alkalmazás, amely célja, hogy az OpenMANIPULATOR robotkart szabad mozgatás mellett precízen lehessen pozicionálni a megadott célpontok fölé.

Az alkalmazás valós és szimulált környezetben is implementálva lett, az alábbi demo videón megtekinthető működés közben.

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

A projekt alapötletét a BME MOGI tanszéken elérhető OpenManipulator robotkar és a hozzá készített telemanipulációs eszköz adta. Ezzel könnyen el lehet végezni egyszerű pick-and-place feladatokat, de a precíz műveletek elvégzése sok időt vesz igénybe, mivel a robotkar átveszi a kézremegéseket is, illetve az eszköz csuklószögeinek mérésekor is kerülhetnek pontatlanságok a rendszerbe.
## Alapelv
Ennek megoldására készítettünk egy alkalmazást, amely az előzetesen meghatározott targetek közelében korlátozza a robot mozgását. Ehhez definiálunk egy-egy kúpot a célok feletti térrészben, és ha a megfogó a kúp által kijelölt térbe kerül, az vezetni fogja a megfogót lefelé mozgásnál. A kúp csúcsa alatt csak függőleges mozgás megengedett. A robotkarra vonatkozó kényszermozgás egy henger alakú téren belül érvényesül, amelyet a kúpok alaplapja és az asztal határol, ahogy az alábbi ábra mutatja. A robotkar mozgásának módosításáért a `coordinator.py` kódban létrehozott node felel. A megfogást fizikai szimulációval, a súrlódások modellezésével valósítottuk meg.

  ![alt text][image1]  
## Gazebo világ
![alt text][image2]

A szimulált világban a robotkart a Gazebo egyszerű asztalmodelljére helyeztük. A manipulálható objektumok egyedi meshsel rendelkező, kockával modellezett objektumok, ezeket a `kocka.urdf.xacro` fájl írja le, a világba a robottal együtt az `open_manipulator_gazebo.launch` fájl helyezi őket.

Ezen kívül elhelyeztünk egy asztalt néző RGB-D kamerát is, ez piros kockaként jelenik meg. A kamera a robot URDF fájljához van hozzáadva.
## RViz megjelenítés
![alt text][image3]

RVizben megjeleníthetjük, mit "lát" a kamera, illetve itt jelennek meg a pozicionáló kúpok is. Az interaktív markeres verzérlés is innen valósítható meg. Ha aktív az aim assist, akkor a megfogó általunk megadott helyzetét sárga marker mutatja, a zöld marker pedig a valós, javított helyzetet. (A megfogó nyitását-zárását az OpenManipulator vezérlő GUI-jával irányíthatjuk)

A robotkart irányíthatjuk ugyanúgy a vezérlő GUI-ból, vagy a `keyboard_controller.py` node használatával, a célkoordináták megadásával.
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
### Irányítás interaktív markerrel

A telemanipulátor interaktív markeres vezérléshez az előző lépéseket egy launch fájlban helyeztük el, azzal a különbséggel, hogy a `contoller_interactive_marker.py` szkriptet futtatjuk le a végén
```
roslaunch telemanipulator_aim_assist telemanipulator_aim_assist.launch
```


### Irányítás MOGI haptikus eszközzel valós környezetben

Indítsuk el a robotvezérlőt és az RVizt valós környezet mellett
```
roslaunch telemanipulator_aim_assist start_real_device.launch
```
Indítsuk el a telemanipulátor vezérlőjét is
```
roslaunch telemanipulator_aim_assist controller_haptic_device.launch
```

<!---## TODO

- végleges szimulált környezet elkészítése
- rviz integráció
- megfogás szimulálása
- saját vezérlő készítése
- mozgatási korlátok és aim assist implementálása
- package.xml testreszabása 
-->

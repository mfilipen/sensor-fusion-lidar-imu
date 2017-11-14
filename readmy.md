# Sesor fusion

Данный репозиторий содержит слудующие packages 

 * racecar_description - для отрисовки размерной модели машины в rviz
 * razor_imu - для визуализации данных с [9 Degrees of Freedom - Razor IMU](https://www.sparkfun.com/products/retired/10736)
 * sensor_fusion - для обьединенпия лидарной одометрии и данных с магнетрометра

Основная идея.  Пакет [hector_mapping](http://wiki.ros.org/hector_mapping) на основе [2д лидара](http://wiki.ros.org/hokuyo_node) предоставляет двумерную карту и одометрию (x,y,yaw). При изменение положения отслеживаемого обьекта по оси z алгоритм теряет свое положение. Идея использования дополнительной информации с других сенсоров представляется интересной. Для начала попробуем сфьюзить данные о положении и ориентациии в дмурной карты лидара (x,y,yaw) и yaw c IMU. 

## Первая попытка
### Запуск
#### Консоль 1
Run ROS master
```bash
roscore
```

#### Консоль 2
Run bag file with dataset.
```bash
cd ./sensor_fusion
rosbag play 2017-11-12-20-18-43.bag
```
#### Консоль 3
Run Rviz for visualization.
```bash
rosrun rviz rviz
```

#### Консоль 4
Download model for vizualization.
```bash
roslaunch racecar_description racecar_model.launch
```
#### Консоль 5
Download vizualization for IMU and for car orientation.
```bash
roslaunch razor_imu razor-display.launch
```
#### Video. Resaults of first run. 



### Stuff used to make this:

 * [markdown-it](https://github.com/markdown-it/markdown-it) for Markdown parsing
 * [CodeMirror](http://codemirror.net/) for the awesome syntax-highlighted editor
 * [highlight.js](http://softwaremaniacs.org/soft/highlight/en/) for syntax highlighting in output code blocks
 * [js-deflate](https://github.com/dankogai/js-deflate) for gzipping of data to make it fit in URLs


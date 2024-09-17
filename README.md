# Robot_Description_Generator

## Установка

Скачаваем с последней версии релиза `.deb` и устанавливаем 
 ```bash
 wget https://github.com/MOShka78/URDF_Expansion/releases/download/0.0.1/ros-humble-urdf-expansion_0.0.1-0jammy_amd64.deb
```

```
 sudo dpkg -i ros-humble-urdf-expansion_0.0.1-0jammy_amd64.deb
 ```

 ## Запуск
1) Создание пакета в директории, где находится файл `urdf`
```bash
ros2 run robot_description_generator robot_description_generator file_urdf.urdf
```
2) Создание пакета с указанием пути 
```bash
ros2 run robot_description_generator robot_description_generator file_urdf.urdf /workspace/packages
```

 ## Использование
 Программа автоматически генерирует пакет `descriprion` под `ros2/ros` используя ваш `urdf` полученный из `SW2URDF`.

 Генерируемый пакет имеет следующую структуру:
```Markdown
.
│your_name_description/
├── config/
│   ├── joint_limits.yaml
│   └── link_mass.yaml
├── launch/
│   ├── your_name_description.launch    // for ros
│   └── your_name_description.launch.py // for ros2
├── meshes/
│   ├── visual/
│   │   └── mesh.STL                    // Одинаковые в двух папках (просто копируется из SW2URDF)
│   └── collision/
│   │   └── mesh.STL
├── urdf/
│   ├── inc/
│   │   └── your_name_property.xacro
│   ├── your_name_macro.xacro
│   └── your_name.urdf.xacro
├── CMakeLists.txt
└──  package.xml
```

> Если после генерации выходит ошибка - проверьте на коррекстость экспортированный `urdf`. Проверить его можно с помощью следующей команды.

```
check_urdf file.urdf
```
> Так же стоит проверить, что у всех `joint` тип которых не `fixed` заданы `axis`.


## Пример

### BU015X
Данный пакет был автоматически сгенерирован `SW2URDF`.
 ```bash
ros2 run robot_description_generator robot_description_generator example/BU015X/urdf/BU015X.urdf /home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/example


---
[INFO] [1726574309.725712326] [robot_description_generator]: Copied file: "link2.STL"
[INFO] [1726574309.726468370] [robot_description_generator]: Copied file: "link1.STL"
[INFO] [1726574309.726822569] [robot_description_generator]: Copied file: "link5.STL"
[INFO] [1726574309.727257414] [robot_description_generator]: Copied file: "link7.STL"
[INFO] [1726574309.727984222] [robot_description_generator]: Copied file: "base_link.STL"
[INFO] [1726574309.728626507] [robot_description_generator]: Copied file: "link3.STL"
[INFO] [1726574309.728936279] [robot_description_generator]: Copied file: "link4.STL"
[INFO] [1726574309.729082162] [robot_description_generator]: Copied file: "link6.STL"
[INFO] [1726574309.729504520] [robot_description_generator]: Create package: /home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/example/BU015X_description/

 ```
 После ввода команды автоматически генерируется пакет `BU015X_description`, который переносим в свой проект.


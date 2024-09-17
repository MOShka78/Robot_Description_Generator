# URDF_Expansion

## Установка

Скачаваем с последней версии релиза `.deb` и устанавливаем 
 ```bash
 wget https://github.com/MOShka78/URDF_Expansion/releases/download/0.0.1/ros-humble-urdf-expansion_0.0.1-0jammy_amd64.deb
```

```
 sudo dpkg -i ros-humble-urdf-expansion_0.0.1-0jammy_amd64.deb
 ```

 ## Запуск

 ```bash
 ros2 run urdf_expansion urdf_expansion file_urdf.urdf
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
 ros2 run urdf_expansion urdf_expansion BU015X.urdf
 ```
 После ввода команды автоматически генерируется пакет `BU015X_description`, который переносим в свой проект.


# PP Finalproject - Group6 : Badminton Courtline Detection
**This project is motified based on [tennis-court-detection](https://github.com/gchlebus/tennis-court-detection.git)**, so most of the content is named as "Tennis..." instead of "Badminton..."

## Environment
![](https://hackmd.io/_uploads/S1ZyYBeF3.png)
![](https://hackmd.io/_uploads/BkKDIHeK3.png)
![](https://hackmd.io/_uploads/rJCvIreY3.png)


## Clone the repository
```
$ git clone https://github.com/maomao726/PP_Finalproject_Group6.git
$ cd {repo}/{version}
```

## Install CMake
`$ sudo apt install cmake`

## Library management packages
`$ pip3 install conan`

:exclamation: conan doesn't support **cmake** generator since version 2.0, which is replaced by **CMakeDeps** + **CMakeToolchain**.

**The previous version, 1.55.0, works for us:**

`$ pip3 install conan==1.55.0`

### Profile example
![](https://hackmd.io/_uploads/SkdrYreKn.png)

## (Optional) Edit conanfile
```
    $ vim conanfile.txt
```
* Change opencv version to newer one (original version, 3.4.3, is too old. ) and generators
![](https://hackmd.io/_uploads/SyijZ4lY2.png)

:exclamation: Not sure if this project is compatible with opencv4.
## (Optional) Create conan profile
* If you lost your conan profile, you can create a new one with this command: 

```
$ conan profile new default --detect    // version 1.X
$ conan profile detect    // version 2.X
```
* You can check your profile with this command:
`$ conan profile show {profileName}`

## (Optional) Edit package_manager
* if you got an error like this while installing packages: 
  ![](https://hackmd.io/_uploads/HJqNpNlFn.png)
Try edit your configurations in your profile, permit conan to update/install the dependencies:
```
$ vim {path to your conan profile}    // default : ~/.conan/profiles/default

add these lines in then save:
  [conf]
  tools.system.package_manager:mode = install
  tools.system.package_manager:sudo = True
```

## Build
```
$ sh build.sh
or
$ mkdir build && cd build
$ conan install .. --build missing
$ cmake ..
$ cmake --build .
```
If you run these instructions successfully, there should be a executable file `detect` under the directory `build/bin`. 

* if you get an error like this: 
![](https://hackmd.io/_uploads/SJx_ofBeF2.png)
that's probably because your path isn't correct. Find where your `conanbuildinfo.cmake` and `CMakeList.txt` at, update the correct path in `CMakeList.txt`, and cmake again. If nothing else go wrong, this step should simultaneouly solve `Unknown Cmake command "conan_basic_setup"` error.

* if you get an error like:
![image](https://hackmd.io/_uploads/ByilKwbK6.png)

    ```$ sudo apt install libcanberra-gtk-module libcanberra-gtk3-module``` could help

## Usage
`$ ./detect -i {image} -f -p {num_of_threads} white`

Descriptions can be found while command is imcomplete
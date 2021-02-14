# K-AR_HYU_Deform_Simulation
ETRI 지원사업 K-AR 과제 시뮬레이션 파트입니다.

## Target platform
Ubuntu (18.04)

## How to build
1. Dependencies 설치
```
> cd src
> install install_dependencies
> install install_dependencies2
```
2. Build
```
make
```
## How to run
```
cd src
make run
```

## Third-party 
시뮬레이션에 쓰이는 tetrahedral 파일 형식은 tet 이며 이 파일을 생성하기 위해서는 tetgen 이 필요합니다

## Make tetrahedral
```
cd src/makeTet
python objtopoly.py target.obj
tetgen -pq target.poly
python nodetotet.py target.1
```
## License
Distributed under the MIT License. See LICENSE for more information.

## Contact
Send to email 
gestoru@hanyang.ac.kr

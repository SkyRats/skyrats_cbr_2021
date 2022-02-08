# skyrats_cbr_2021
Repositório destinado aos desenvolvimentos para a Competição Brasileira de Robótica de 2021 feitos pela Equipe Skyrats de Drones Autônomos da Poli-USP, que levaram a equipe ao **primeiro lugar** na categoria Desafio Petrobrás.

## Instalações

A partir daqui vc deve ter acabado de instalar Ubuntu 20.04 no seu computador (novinho e folha ;) ). 
Siga os seguintes passos pra fazer o tudo funcionar (deve ter outros jeitos de fazer isso mas desse jeito foi testado e funcionou):

* Dar um ``` sudo apt update ``` e um ``` sudo apt upgrade``` no terminal só pra ter certeza de que tá tudo atualizado.
* Instalar ROS Noetic pelo seguinte [tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu). Você deve ir copiando e colando os comandos no seu terminal do item 1.1 a 1.6. PS: No item Installation (1.4) vc tem que selecionar **só** o Desktop-Full Install.
* Instalar também o python-catkin-tools com ``` sudo apt install python-catkin-tools``` para facilitar na hora de buildar os pacotes.
* Criar um workspace na $HOME com:
    * ``` mkdir -p mavros_ws/src```
    * ``` cd mavros_ws ```
    * ``` catkin init ```
* Copiar o repositorio CORE na src do seu Workspace e instalar as dependencias com:
    * ``` cd mavros_ws/src ```
    * ``` git clone https://github.com/ctu-mrs/uav_core.git ```
    * ``` source uav_core/installation/install.sh```
* Buildar o CORE com:
    * ```cd mavros_ws```
    * ``` catkin build ```
* Copiar o repositorio SIMULATION na src do seu Workspace e instalar as dependencias com:
    * ``` cd mavros_ws/src ```
    * ``` git clone https://github.com/ctu-mrs/simulation.git ```
    * ``` source simulation/installation/install.sh```
* Inicializar simulação da PX4 com:
    * ``` cd simulation/ros_packages/px4_firmware```
    * ``` make px4_sitl gazebo```
    * Rezar pra funcionar
* Buildar tudo com:
    * ```cd mavros_ws```
    * ``` catkin build ```
    * Cruzar os dedos
* Setar as variaveis no terminal com:
```bash: 
    echo "source ~/mavros_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc 
```
* Agora é um pouco mais complicado pq vai precisar mexer em alguns codigos mas nada muito dificil.
    * Instalar dependencias com:
    ```bash:
    sudo apt-get update
    sudo apt-get install libfftw3-dev
    sudo apt-get install libclfft-dev
    sudo apt-get install libcgal-dev
    ```
    * Copiar o repositorio da competição na workspace com:
        * ``` cd mavros_ws/src ```
        * ``` git clone https://github.com/LASER-Robotics/Drone_Trial_League.git ```
    * Seguir este [tutorial](https://github.com/LASER-Robotics/Drone_Trial_League/tree/master/gripper) dentro da pasta gripper do Drone_Trial_League. È basicamente ficar copiando linha de codigo e colando em outro arquivo no repositorio simulation que foi instalado antes.
* Por fim (ufaaa) é só buildar tudo com:
    * ```cd mavros_ws```
    * ``` catkin build ```
* Ultima coisinha mas tbm muito importante pra fazer a garra funcionar:
    * ```sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers```
* Pronto!! Agora é só testar se está funcionando com:
    * ``` cd  mavros_ws/src/Drone_Trial_League/offshore_uav_pack/start ```
    * ``` source start.sh ```

## Como rodar ?
Para rodar os códigos basta entrar na pasta correspondente com a fase que você deseja rodar na pasta tmux_scripts pelo terminal. Lá dentro use: ```./start```

Isso deve iniciar a simulação. 

Agora, para iniciar a movimentação do drone, use o roslaunch na missão desejada. Como por exemplo, para rodar a versão final da fase 1, o usuário deve digitar:

```roslaunch skyrats_cbr_2021 fase1.launch```

Os diversos arquivos ```.launch``` podem ser encontrados na pasta ```/launch``` desse repositório

### Notas importantes
* O ```./start``` da fase 2 já inicia a missão, sem necessidade de um roslaunch ser rodado.
* Os arquivos launch devem ser rodados assim que o drone se estabilizar no ar, uma vez que nas fases 3 e 4 as bases móveis podem sair muito de suas posições iniciais caso o usuário demore para rodar o comando, o que resultaria no drone não achando essas bases.
# Edit simulation
Este arquivo é destinado a mudança da simulação ``` Aruco.world ```, a fim de ser um mundo mais viável para testes, lembre de não dar commit nessas edições pessoais.
 
## editar o mundo
Entre na pasta e rode:
```bash:
    cd skyrats_ws2/src
    code simulation2
```
Abra o ``` Aruco.world ``` e edite os modelos, um modelo de exemplo é:
 
```
<include><!-- Base da cbr -->
      <static>true</static>
      <uri>model://cbrBaseground1</uri>
      <name>baseCBR1</name>
      <pose>5 4 1.5 0 0 0</pose>
</include>
```
 
O mesmo modelo pode ser usado várias vezes, mas é importante que cada um tenha um nome
Os valores dentro de ``` <pose> ``` são, os eixos **x, y e z**, associados a **5, 4, 1.5**, respectivamente.
Caso você queira algum modelo diferente, basta editar o ``` <uri> ``` para algum desejado, como ``` <uri>model://cbrMainBase</uri> ``` ou ``` <uri>model://QrcodeA</uri> ```, alguns deles não spawnam no centro do mundo, mas sempre da pra rodar o mundo e ver para onde alterar.
Voces podem tirar 
Salve o mundo.
 
## buildar o mundo
Rode:
```bash:
    cd
    cd skyrats_ws2
    colcon build --packages-select simulation2
```
 
## rodar o mundo
Para ver o mundo editado e testar o drone, rode:
```bash:
    cd
    cd skyrats_ws2/src/simulation2/scripts
    bash simulate.sh
```
 
escolha o mundo 1.
Caso tenham alguma dúvida, podem entrar em contato com o Pereira.

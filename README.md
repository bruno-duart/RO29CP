# RO29CP

Atividades desenvolvidas na disciplina Robótica Móvel, do curso de Engenharia de Computação da UTFPR campus Pato Branco.

## Organização dos Arquivos

O diretório /catkin_ws refere-se ao *workspace* do ambiente ROS.

A disciplina é estruturada utilizando as linguagens de programação C++ e Python. Este repositório contém somente códigos em Python, localizados na pasta /catkin/src.

O script para o exercício 1 da disciplina pode ser encontrado em /catkin/src/my_robot/script, com o nome **motor_control_node.py**.

O script para o exercício 2 da disciplina pode ser encontrado em /catkin/src/interact_two_tb3/script, com o nome **control_two_tb3.py**.

O script para o exercício 3 da disciplina pode ser encontrado em /catkin_ws/src/desafio1_tb3_siga_o_mestre/src/script/, com o nome **position_control_TB3.py**.

## Requisitos

<ul>
    <li> Sistema operacional Linux - Meu SO é o Ubuntu 20.04 LTS </li>
    <li> Ambiente de programação Python (preferencialmente a partir do Python 3.6) </li>
    <li> Instalação do Ambiente ROS</li>
</ul>

## Instalação e Uso

Para tutorial de instalação do ROS, configuração de workspace e criação de pacotes, seguir o passo a passo no [vídeo](https://www.youtube.com/watch?v=Fg-58etXqeo&list=PL55DJi5ukTqtD9UyH_XhRba23xZykOYdb&index=1&ab_channel=JefersonLima).

Para executar o desafio 1 seguir os seguintes passos no terminal Linux:

    $ cd ~/catkin_ws
    $ roslaunch desafio1_tb3_siga_o_mestre position_control_TB3.py
    $ roslaunch desafio1_tb3_siga_o_mestre two_tb3.launch
    

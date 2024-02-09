# Controllo del Robot Panda con ROS

Questo repository contiene tre file principali che gestiscono il controllo del robot Panda utilizzando il framework ROS (Robot Operating System). Di seguito è fornita una spiegazione dettagliata di ciascun file.

# Utilizzo del Progetto

Questi file vanno inseriti all'interno di un Package Ros, quindi installare correttamente Ros [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) ed seguire il tutorial di Movit per l'installazione base [Movit](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

## ATTENZIONE arrivati all'istruzione catkin build

`` `bash
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
`` `

Nel SRC creare un Package con questi file, ecco la guida ufficiale [Creazione di un ROS Package](https://wiki.ros.org/it/ROS/Tutorials/CreatingPackage)



## File fsm.py

Il file fsm.py implementa una macchina a stati finiti (FSM) utilizzando il framework smach di ROS. La FSM gestisce il comportamento del robot Panda in base a diversi stati interni. I principali componenti del file sono:

### Classi di stato:

- WAIT: Rappresenta lo stato di attesa in cui il robot si trova prima di iniziare un'azione.
- GOTO: Gestisce lo spostamento del robot verso una posizione specifica.
- PICK: Si occupa del processo di raccolta di un oggetto.
- PLACE: Gestisce il processo di posizionamento di un oggetto raccolto.

### Callback:

- Callback_start, Callback_stop, Callback_picked, Callback_target: Callbacks per i topic ROS che aggiornano lo stato della FSM in base agli input ricevuti.

### ROS Node:

- State_Machine: Inizializza il nodo ROS e i subscriber per i topic relativi agli input della FSM.

### Main:

- La funzione main() avvia la FSM e il suo esecutore, gestendo le transizioni di stato.

## File control.py

Il file control.py implementa un nodo ROS che gestisce le comunicazioni tra la macchina a stati (fsm.py) e il robot Panda. I principali componenti del file sono:

### Classi:

- ControlNode: Rappresenta il nodo ROS che controlla il flusso del lavoro del robot Panda.

### Publisher:

- **target_arm** (Pose): Questo publisher trasmette la posizione desiderata del braccio del robot Panda, indicando dove posizionare l'end effector. Viene utilizzato per inviare le posizioni di prelievo e rilascio dell'oggetto.
- **endeffector_status**(Point): Trasmette lo stato dell'end effector del robot, indicando se deve essere aperto o chiuso. Viene utilizzato per controllare la presa e il rilascio dell'oggetto.
- **status_job**(Bool): Questo publisher segnala se il braccio del robot Panda è impegnato in un'attività specifica, indicando se è occupato o disponibile per nuovi compiti.

### Subscriber:

- **/status**(String): Questo subscriber legge lo stato corrente della macchina a stati, ricevendo informazioni sullo stato in cui si trova la FSM. È utilizzato per coordinare il flusso di lavoro del robot Panda in base agli stati della FSM.
- **status_job**(Bool): Legge lo stato del lavoro del braccio del robot, informando se è attualmente impegnato in un compito specifico o se è disponibile per nuove attività.

### Servizi:

- **start**: Questo servizio è gestito da un monitor per la sicurezza e consente di avviare il controllo del robot Panda, permettendo al nodo `control.py` di iniziare il coordinamento del flusso di lavoro.
- **stop**: Gestito dallo stesso monitor, questo servizio permette di interrompere il controllo del robot Panda, fermando tutte le attività in corso e riportando il sistema allo stato di attesa.

Queste funzionalità consentono a `control.py` di gestire in modo efficiente il flusso di lavoro del robot Panda, coordinando le azioni in base agli stati della macchina a stati e ai compiti da eseguire.

## File panda.py

Il file panda.py implementa un nodo ROS per controllare il braccio del robot Panda utilizzando il framework MoveIt. I principali componenti del file sono:

### Classi:

- ArmController: Rappresenta il controller del braccio del robot Panda.

### Metodi:

- move_controller: Gestisce il movimento del braccio e dell'end effector del robot in base ai target ricevuti.
- move_arm, move_eef: Eseguono rispettivamente il movimento del braccio e dell'end effector utilizzando MoveIt.

## File panda_arm_hand.urdf.xacro

[panda_arm_hand.urdf.xacro](https://github.com/StanfordASL/PandaRobot.jl/blob/master/deps/Panda/panda.urdf)
Questo file URDF definisce la struttura del robot Panda, compresi i link e le articolazioni che compongono il braccio e la mano del robot. In particolare, sono stati creati due gruppi separati, uno per panda_hand e uno per panda_arm, per facilitare il controllo e la pianificazione del movimento dell'end effector e del braccio.


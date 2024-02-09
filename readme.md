# Controllo del Robot Panda con ROS

Questo repository contiene tre file principali che gestiscono il controllo del robot Panda utilizzando il framework ROS (Robot Operating System). Di seguito è fornita una spiegazione dettagliata di ciascun file.

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

### Callback:

- start_callback, stop_callback: Callbacks per i servizi ROS che gestiscono le richieste di avvio e di arresto del lavoro del robot.

### Metodi:

- start_action, stop_action: Gestiscono rispettivamente l'avvio e l'arresto delle azioni del robot Panda.

## File panda.py

Il file panda.py implementa un nodo ROS per controllare il braccio del robot Panda utilizzando il framework MoveIt. I principali componenti del file sono:

### Classi:

- ArmController: Rappresenta il controller del braccio del robot Panda.

### Metodi:

- move_controller: Gestisce il movimento del braccio e dell'end effector del robot in base ai target ricevuti.
- move_arm, move_eef: Eseguono rispettivamente il movimento del braccio e dell'end effector utilizzando MoveIt.

## File panda_arm_hand.urdf.xacro

Questo file URDF definisce la struttura del robot Panda, compresi i link e le articolazioni che compongono il braccio e la mano del robot. In particolare, sono stati creati due gruppi separati, uno per panda_hand e uno per panda_arm, per facilitare il controllo e la pianificazione del movimento dell'end effector e del braccio.


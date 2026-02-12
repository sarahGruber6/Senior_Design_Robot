*These steps are assuming you are on a Windows computer with admin privileges, if you're on Linux I expect you to know how to figure it out, lol!*



**how to open MQTT broker manually:**

&nbsp;	1. navigate to mosquitto folder and run mosquitto.exe

&nbsp;	  $ cd ..\\Server\\Mosquitto

&nbsp;	  $ .\\mosquitto.exe

&nbsp;	done! to test broker, open two shells in mosquitto directory:

&nbsp;	  shell 1: $ .\\mosquitto\_sub.exe -t "test/topic"

&nbsp;	  shell 2: $ .\\mosquitto\_pub.exe -t "test/topic" -m "hello"

&nbsp;	  "hello" should print in shell 1



**opening and testing the publishing api:**

&nbsp;	1. activate the virtual environment!

&nbsp;	  $ .\\.venv\\Scripts\\activate

&nbsp;	2. then run the server

&nbsp;	  $ set MQTT\_HOST=127.0.0.1

&nbsp;	  $ python -m Server.app

&nbsp;	

&nbsp;	From here you can test many things! Jobs are sent to a .db file in the Server/job\_db folder, you will have to download SQLite Browser to open and view this, which you can do here: [https://sqlitebrowser.org/dl/](https://sqlitebrowser.org/dl/)

Similar to WinRAR; download the zip, unzip, then open the .db file with the exe found in the unzipped file.

&nbsp;	I've also made an endpoint on the server where you can just view the json of all the jobs in queue if you don't want to have to use SQLite. The archive button renames the current file to the date/time and sends it to the /archive/ folder. 

&nbsp;		






*These steps are assuming you are on a Windows computer with admin privileges, if you're on Linux I expect you to know how to figure it out, lol!*

**How to open MQTT broker:**

1. navigate to parent folder and run mosquitto.exe 

      $ ```.\Server\Mosquitto\mosquitto.exe```

2. done! to test broker, you can open two shells:

      shell 1: $ ```.\Server\Mosquitto\mosquitto_sub.exe -t "test/topic"```

      shell 2: $ ```.\Server\Mosquitto\mosquitto_pub.exe -t "test/topic" -m "hello"```

      _"hello" should print in shell 1_


**Opening and testing the flask server:**

1. install python on your computer! i use 3.12 but any newer version should work

2. install virtual environment

      $ ```python -m venv .venv```

3. activate venv

      $ ```.\Server\.venv\Scripts\active```

4. install dependencies

      $ ```pip install -r .\Server\requirements.txt```

5. run the server!

      $ ```python -m Server.app```

From here you can test many things! Jobs are sent to a .db file in the Server/job\_db folder, you will have to download SQLite Browser to open and view this, which you can do [here](https://sqlitebrowser.org/dl/).
Installing is similar to WinRAR: Download the zip, unzip, then open the .db file with the exe found in the unzipped file.

 


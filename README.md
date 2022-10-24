# My Service Test
This package is for checking behavior of service serve/client with composed node.


## Run
### Server/Client composed node
```bash
ros2 launch my_service_test composed.launch.py
```

Then service client never get the response from the server.


### Server/Client separated node
```bash
ros2 launch my_service_test separated.launch.py
```

Then service client can get the response (`STATUS: [Success] Message: [Hello World1]` appears on console.)

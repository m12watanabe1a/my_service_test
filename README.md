# My Service Test
This package is for checking behavior of service serve/client with composable node.


## Run
### Server/Client composed node
```bash
ros2 launch my_service_test composed.launch.py
```

Then service client never get the response from the server.

### Server/Client composed node with multi-thread
```bash
ros2 launch my_service_test composed_mt.launch.py
```

Then service client can get the response (`STATUS: [Success] Message: [Hello World1]` appears on console.)


### Server/Client separated node
```bash
ros2 launch my_service_test separated.launch.py
```

Then service client can get the response (`STATUS: [Success] Message: [Hello World1]` appears on console.)

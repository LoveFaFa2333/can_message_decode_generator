# can_message_decode_generator
# Description
  Generate CAN message decode code from dbc file.
# How to use
### 1. Generate decode hpp file
(1) Put the dbc file in directory <can_message_cpp_generator/dbc/>

(2) Modify the namespace_name and dbc file name in python file(can_message_cpp_generator.py) as followings:
```python
namespace_name = <your_namespace_name>
db_can = cantools.db.load_file("can_message_cpp_generator/dbc/<your_dbc_file_name>.dbc")
```
(3) Launch python file to generate the decode code

(4) Find the decode code in directory <output> 

### 2. Use decode hpp file
you can refer to the test file in the directory <can_message_decode/decode_test.cpp> 

'''
Descripttion: CAN message cpp file generator
Author: Gang Wang
Date: 2023-01-30 09:30:40
'''
import cantools

def canMessageHppGenerator():
    namespace_name = 'gnss_ins'
    db_can = cantools.db.load_file("can_message_cpp_generator/dbc/gnss_ins_motorola.dbc")
    message_num = len(db_can.messages)

    # 1. 生成文件头
    if(db_can.messages[0].signals[0].byte_order == 'little_endian'):
        hpp_name = namespace_name + '_can_msg_decode_intel.hpp'

    else:
        hpp_name = namespace_name + '_can_msg_decode_motorola.hpp'

    output_dir = 'can_message_cpp_generator/output/' + hpp_name
    hpp_file = open(output_dir, 'w')

    # 2. 定义文件头
    hpp_file.write('#ifndef _' + namespace_name.upper() + '_CAN_MESSAGE_DECODE_HPP_\n')
    hpp_file.write('#define _' + namespace_name.upper() + '_CAN_MESSAGE_DECODE_HPP_\n\n\n')

    # 3. include 清单
    hpp_file.write('#include "can_message/can_message.hpp"\n')
    hpp_file.write('#include "can_message/can_message.hpp"\n')
    hpp_file.write('#include "can_message/signal.hpp"\n\n')
    hpp_file.write('#include "utils/utils.hpp"\n\n')


    # 4. message frame id 定义
    hpp_file.write('/* Frame ids. */\n')
    for i in range(message_num):
        can_message = db_can.messages[i]
        hpp_file.write('#define ' + can_message.name.upper() + '_FRAME_ID (' + hex(can_message.frame_id) +'u)\n' )
    hpp_file.write("\n\n")
    hpp_file.write('/************ Message Class Defination **********/ \n\n')

    # 5. 定义namespace
    hpp_file.write('namespace ' + namespace_name + '{\n')
    hpp_file.write("\n")

    # 6. 定义message 类
    for i in range(message_num):
        can_message = db_can.messages[i]
        # print(can_message)
        # print(can_message.signals)
        signals_num = len(can_message.signals)
        message_name = can_message.name
        senders = ""
        for i in range(len(can_message.senders)):
            senders += can_message.senders[i]

        hpp_file.write('class ' + message_name + ' : public CanMessage{\n')
        hpp_file.write('public: \n')

        hpp_file.write('\t// 构造函数\n')
        hpp_file.write('\t' + message_name + '(){\n')
        hpp_file.write('\t\tthis->frame_id = ' + message_name.upper() + '_FRAME_ID' + ';\n')
        hpp_file.write('\t\tthis->message_length = ' + str(can_message.length * 8) + ';\n')
        hpp_file.write('\t\tthis->node_name = {' + '"' + senders + '"' +'};\n')
        hpp_file.write('\t\tthis->data = 0;\n')
        hpp_file.write('\t\tthis->data_bin = _data_bin_type(this->data);\n')
        hpp_file.write('\t\tthis->decode();\n')
        hpp_file.write('\t}\n')


        hpp_file.write('\t// 构造函数\n')
        hpp_file.write('\texplicit ' + message_name + '(const _data_type& message_data){\n')
        hpp_file.write('\t\tthis->frame_id = ' + message_name.upper() + '_FRAME_ID' + ';\n')
        hpp_file.write('\t\tthis->message_length = ' + str(can_message.length * 8) + ';\n')
        hpp_file.write('\t\tthis->node_name = {' + '"' + senders + '"' +'};\n')
        hpp_file.write('\t\tthis->data = message_data;\n')
        hpp_file.write('\t\tthis->data_bin = _data_bin_type(this->data);\n')
        hpp_file.write('\t\tthis->decode();\n')
        hpp_file.write('\t}\n')

        hpp_file.write('\t// 构造函数\n')
        hpp_file.write('\texplicit ' + message_name + '(const _frame_id_type& id){\n')
        hpp_file.write('\t\tthis->frame_id = id;\n')
        hpp_file.write('\t\tthis->message_length = ' + str(can_message.length * 8) + ';\n')
        hpp_file.write('\t\tthis->node_name = {' + '"' + senders + '"' +'};\n')
        hpp_file.write('\t\tthis->data = 0;\n')
        hpp_file.write('\t\tthis->data_bin = _data_bin_type(this->data);\n')
        hpp_file.write('\t\tthis->decode();\n')
        hpp_file.write('\t}\n')

        hpp_file.write('\t// 构造函数\n')
        hpp_file.write('\texplicit ' + message_name + '(const _frame_id_type& id , const _data_type& message_data){\n')
        hpp_file.write('\t\tthis->frame_id = id;\n')
        hpp_file.write('\t\tthis->message_length = ' + str(can_message.length * 8) + ';\n')
        hpp_file.write('\t\tthis->node_name = {' + '"' + senders + '"' +'};\n')
        hpp_file.write('\t\tthis->data = message_data;\n')
        hpp_file.write('\t\tthis->data_bin = _data_bin_type(this->data);\n')
        hpp_file.write('\t\tthis->decode();\n')
        hpp_file.write('\t}\n')

        hpp_file.write('/************************ Signals Defination *****************************/\n')
        # 7. 定义信号类
        signal_name = []
        for j in range(signals_num):
            signal = can_message.signals[j]
            signal_name.append(signal.name)
            signal_start_bit = signal.start
            signal_is_unsigned = (signal.is_signed == False)
            signal_length = signal.length
            signal_is_intel = ('true' if signal.byte_order == 'little_endian' else 'false')
            signal_scale = signal.scale
            signal_offset = signal.offset
            signal_max = signal.maximum
            signal_min = signal.minimum
            signal_unit = "" if signal.unit == None else signal.unit
            signal_receiver = signal.receivers
            
            # 获取信号的数值类型
            signal_data_type = {
                    0 : "int8_t",
                    1 : "int16_t",
                    2 : "int32_t",
                    3 : "int32_t",
                    4 : "int64_t",
                    5 : "int64_t",
                    6 : "int64_t",
                    7 : "int64_t",
                    8 : "uint8_t",
                    9 : "uint16_t",
                    10 : "uint32_t",
                    11 : "uint32_t",
                    12 : "uint64_t",
                    13 : "uint64_t",
                    14 : "uint64_t",
                    15 : "uint64_t",

                }
            num = (int)((signal_length-1) / 8) + (0 if (signal_is_unsigned == False) else 8)
            signal_data_type = signal_data_type.get(num , None)

            #获取信号真实值的数据类型
            signal_scale_type = 'double' if isinstance(signal_scale,float) == True else 'int64_t'

            hpp_file.write('\tclass ' + signal_name[j] + ' : public Signal<' + signal_data_type + ' , ' + signal_scale_type + '>{\n')
            hpp_file.write('\tpublic: \n')

            hpp_file.write('\t\t// 构造函数\n')
            hpp_file.write('\t\t' + signal_name[j] + '(){\n')
            hpp_file.write('\t\t\tthis->data = ' + "0" + ';\n')
            hpp_file.write('\t\t\tthis->is_unsigned_ = ' + str(signal_is_unsigned).lower() + ';\n')
            hpp_file.write('\t\t\tthis->start_bit_ = ' + str(signal_start_bit) + ';\n')
            hpp_file.write('\t\t\tthis->signal_length_ = ' + str(signal_length) + ';\n')
            hpp_file.write('\t\t\tthis->is_intel_ = ' + signal_is_intel + ';\n')
            hpp_file.write('\t\t\tthis->scale_ = ' + str(signal_scale) + ';\n')
            hpp_file.write('\t\t\tthis->offset_ = ' + str(signal_offset) + ';\n')
            hpp_file.write('\t\t\tthis->range_min_ = ' + ('0' if str(signal_min) == 'None' else str(signal_min)) + ';\n')
            hpp_file.write('\t\t\tthis->range_max_ = ' + ('0' if str(signal_max) == 'None' else str(signal_max)) + ';\n')
            hpp_file.write('\t\t\tthis->unit_ = ' + '"' + signal_unit + '"' + ';\n')

            hpp_file.write('\t\t\tthis->receiver_ = {')
            for k in range(len(signal_receiver)):
                if k != len(signal_receiver)-1:
                    hpp_file.write('"' + str(signal_receiver[k]) + '" , ')
                else:
                    hpp_file.write('"' + str(signal_receiver[k]) + '"')
            hpp_file.write('};\n')
            
            hpp_file.write('\t\t}\n')
            hpp_file.write('\t};\n')
            hpp_file.write('\n')
        hpp_file.write('/*************************** END Defination ******************************/\n')
        hpp_file.write('\n')

        # 8. 重写encode函数
        hpp_file.write('public: \n')
        hpp_file.write('\tvirtual void encode() override{\n')
        hpp_file.write('\t\t// signals encode\n')

        if signal_is_intel == 'true':
            for j in range(len(signal_name)):
                hpp_file.write('\t\t// ' + str(j+1) +'. ' + signal_name[j] + '\n')
                hpp_file.write('\t\t{\n')
                hpp_file.write('\t\t\t_data_bin_type ' + signal_name[j].lower() +'_data_bin(this->' + signal_name[j].lower() + '.data);\n') 
                hpp_file.write('\t\t\tfor(uint8_t i = 0 ; i < this->' + signal_name[j].lower() + '.signal_length() ; i++){\n')
                hpp_file.write('\t\t\t\tthis->data_bin[i + this->' + signal_name[j].lower() +'.start_bit()] = ' + signal_name[j].lower() +'_data_bin[i];\n')
                hpp_file.write('\t\t\t}\n')
                hpp_file.write('\t\t}\n')
        else :
            for j in range(len(signal_name)):
                hpp_file.write('\t\t// ' + str(j+1) +'. ' + signal_name[j] + '\n')
                hpp_file.write('\t\t{\n')
                hpp_file.write('\t\t\t_data_bin_type ' + signal_name[j].lower() +'_data_bin(this->' + signal_name[j].lower() + '.data);\n') 
                hpp_file.write('\t\t\tuint8_t index = this->'+ signal_name[j].lower() +'.start_bit() , length = this->'+ signal_name[j].lower() +'.signal_length() - 1;\n') 
                hpp_file.write('\t\t\tfor(uint8_t i = 0 ; i <= length ; i++){\n') 
                hpp_file.write('\t\t\t\tthis->data_bin[index] = '+ signal_name[j].lower() +'_data_bin[length - i];\n')
                hpp_file.write('\t\t\t\tindex += (index%8 == 0)? 15 : -1;\n')
                hpp_file.write('\t\t\t}\n')
                hpp_file.write('\t\t}\n')

        
        hpp_file.write('\t\tthis->data = this->data_bin.to_ulong();\n')
        hpp_file.write('\t}\n')
        hpp_file.write('\n')

        # 9. 重写decode函数
        hpp_file.write('private: \n')
        hpp_file.write('\tvirtual void decode() override{\n')
        hpp_file.write('\t\t// signals decode\n')
        
        if signal_is_intel == 'true':
            for j in range(len(signal_name)):
                hpp_file.write('\t\t// ' + str(j+1) +'. ' + signal_name[j] + '\n')
                hpp_file.write('\t\t{\n')
                hpp_file.write('\t\t\t_data_bin_type ' + signal_name[j].lower() +'_data_bin;\n') 
                hpp_file.write('\t\t\tfor(uint8_t i = 0; i < this->'+ signal_name[j].lower() +'.signal_length(); i++)\n')
                hpp_file.write('\t\t\t\t'+ signal_name[j].lower() +'_data_bin[i] = this->data_bin[this->'+ signal_name[j].lower() +'.start_bit() + i];\n')
                hpp_file.write('\t\t\tthis->' + signal_name[j].lower() + '.data = utils::bin2int(' + signal_name[j].lower() +'_data_bin, this->' + signal_name[j].lower() + '.signal_length());\n')
                hpp_file.write('\t\t}\n')
        else :
            for j in range(len(signal_name)):
                hpp_file.write('\t\t// ' + str(j+1) +'. ' + signal_name[j] + '\n')
                hpp_file.write('\t\t{\n')
                hpp_file.write('\t\t\t_data_bin_type ' + signal_name[j].lower() +'_data_bin;\n') 
                hpp_file.write('\t\t\tuint8_t index = 0 ,  length = '+ signal_name[j].lower()  +'.signal_length()-1;\n') 
                hpp_file.write('\t\t\tuint8_t j = this->'+ signal_name[j].lower() +'.start_bit();\n') 
                hpp_file.write('\t\t\twhile(index <= length){\n') 
                hpp_file.write('\t\t\t\t'+ signal_name[j].lower() + '_data_bin[length - index] = this->data_bin[j];\n') 
                hpp_file.write('\t\t\t\tj += (j%8 == 0)? 15 : -1;\n') 
                hpp_file.write('\t\t\t\tindex++;\n') 
                hpp_file.write('\t\t\t}\n') 
                hpp_file.write('\t\t\tthis->' + signal_name[j].lower() + '.data = utils::bin2int(' + signal_name[j].lower() +'_data_bin, this->' + signal_name[j].lower() + '.signal_length());\n')
                hpp_file.write('\t\t}\n')

        hpp_file.write('\t}\n')
        hpp_file.write('\n')

        # 10. 定义message的各个信号对象
        hpp_file.write('public: \n')
        for j in range(len(signal_name)):
            hpp_file.write('\t' + signal_name[j] + ' ' + signal_name[j].lower() + ';\n')

        hpp_file.write('};\n\n\n')

    # namespace end
    hpp_file.write('} // namespace ' + namespace_name + '\n')

    # endif
    hpp_file.write("\n")
    hpp_file.write('#endif\n')

if __name__ == "__main__":
    canMessageHppGenerator()


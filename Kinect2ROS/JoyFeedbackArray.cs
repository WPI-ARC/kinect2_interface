using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

//////////////////////////////////////////////////
/////    AUTOGENERATED MESSAGE DEFINITION    /////
//////////////////////////////////////////////////
/////         DO NOT MODIFY BY HAND!         /////
//////////////////////////////////////////////////

namespace ROS_CS
{
    namespace sensor_msgs
    {
        public class JoyFeedbackArray : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "JoyFeedbackArray";
            public readonly string md5sum = "cde5730a895b1fc4dee6f91b754b213d";
            public List<sensor_msgs.JoyFeedback> array;

            public JoyFeedbackArray ()
            {
                array = new List<sensor_msgs.JoyFeedback>();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += "array:\n[";
                foreach (sensor_msgs.JoyFeedback element in array)
                {
                    string_rep += " " + element.ToStringRaw();
                }
                string_rep += "]\n\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                System.Byte[] array_len_bytes = BitConverter.GetBytes((System.UInt32)array.Count);
                stream.Write(array_len_bytes, 0, array_len_bytes.Length);
                foreach(sensor_msgs.JoyFeedback element in array)
                {
                    element.Serialize(stream);
                }
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                System.UInt32 array_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(array_len).Length;
                for (int i = 0; i < (int)array_len; i++)
                {
                    sensor_msgs.JoyFeedback element = new sensor_msgs.JoyFeedback();
                    curIndex += element.Deserialize(serialized, curIndex);
                    array.Add(element);
                }
                return (curIndex - startIndex);
            }

        }
    }
}

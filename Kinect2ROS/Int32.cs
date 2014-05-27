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
    namespace std_msgs
    {
        public class Int32 : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "Int32";
            public readonly string md5sum = "da5909fbe378aeaf85e547e830cc1bb7";
            public System.Int32 data;

            public Int32 ()
            {
                data = 0;
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += "data: " + Convert.ToString(data) + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                System.Byte[] data_bytes = BitConverter.GetBytes(data);
                stream.Write(data_bytes, 0, data_bytes.Length);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                data = BitConverter.ToInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(data).Length;
                return (curIndex - startIndex);
            }

        }
    }
}
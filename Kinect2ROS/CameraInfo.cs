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
        public class CameraInfo : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "CameraInfo";
            public readonly string md5sum = "c9a58c1b0b154e0e6da7578cb991d214";
            public std_msgs.Header header;
            public System.UInt32 height;
            public System.UInt32 width;
            public string distortion_model;
            public List<double> D;
            public double[] K;
            public double[] R;
            public double[] P;
            public System.UInt32 binning_x;
            public System.UInt32 binning_y;
            public sensor_msgs.RegionOfInterest roi;

            public CameraInfo ()
            {
                header = new std_msgs.Header();
                height = 0;
                width = 0;
                distortion_model = "";
                D = new List<double>();
                K = new double[9];
                R = new double[9];
                P = new double[12];
                binning_x = 0;
                binning_y = 0;
                roi = new sensor_msgs.RegionOfInterest();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += header.ToStringRaw() + "\n";
                string_rep += "height: " + Convert.ToString(height) + "\n";
                string_rep += "width: " + Convert.ToString(width) + "\n";
                string_rep += "distortion_model: " + distortion_model + "\n";
                string_rep += "D:\n[";
                foreach (double element in D)
                {
                    string_rep += " " + Convert.ToString(element);
                }
                string_rep += "]\n\n";
                string_rep += "K: " + K + "\n";
                string_rep += "R: " + R + "\n";
                string_rep += "P: " + P + "\n";
                string_rep += "binning_x: " + Convert.ToString(binning_x) + "\n";
                string_rep += "binning_y: " + Convert.ToString(binning_y) + "\n";
                string_rep += roi.ToStringRaw() + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                header.Serialize(stream);
                System.Byte[] height_bytes = BitConverter.GetBytes(height);
                stream.Write(height_bytes, 0, height_bytes.Length);
                System.Byte[] width_bytes = BitConverter.GetBytes(width);
                stream.Write(width_bytes, 0, width_bytes.Length);
                System.Byte[] distortion_model_bytes = System.Text.Encoding.UTF8.GetBytes(distortion_model);
                System.Byte[] distortion_model_len_bytes = BitConverter.GetBytes((System.UInt32)distortion_model_bytes.Length);
                stream.Write(distortion_model_len_bytes, 0, distortion_model_len_bytes.Length);
                stream.Write(distortion_model_bytes, 0, distortion_model_bytes.Length);
                System.Byte[] D_len_bytes = BitConverter.GetBytes((System.UInt32)D.Count);
                stream.Write(D_len_bytes, 0, D_len_bytes.Length);
                foreach(double element in D)
                {
                    System.Byte[] element_bytes = BitConverter.GetBytes(element);
                    stream.Write(element_bytes, 0, element_bytes.Length);
                }
                foreach(double element in K)
                {
                    System.Byte[] element_bytes = BitConverter.GetBytes(element);
                    stream.Write(element_bytes, 0, element_bytes.Length);
                }
                foreach(double element in R)
                {
                    System.Byte[] element_bytes = BitConverter.GetBytes(element);
                    stream.Write(element_bytes, 0, element_bytes.Length);
                }
                foreach(double element in P)
                {
                    System.Byte[] element_bytes = BitConverter.GetBytes(element);
                    stream.Write(element_bytes, 0, element_bytes.Length);
                }
                System.Byte[] binning_x_bytes = BitConverter.GetBytes(binning_x);
                stream.Write(binning_x_bytes, 0, binning_x_bytes.Length);
                System.Byte[] binning_y_bytes = BitConverter.GetBytes(binning_y);
                stream.Write(binning_y_bytes, 0, binning_y_bytes.Length);
                roi.Serialize(stream);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += header.Deserialize(serialized, curIndex);
                height = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(height).Length;
                width = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(width).Length;
                System.UInt32 distortion_model_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(distortion_model_len).Length;
                distortion_model = System.Text.Encoding.UTF8.GetString(serialized, curIndex, (int)distortion_model_len);
                curIndex += (int)distortion_model_len;
                System.UInt32 D_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(D_len).Length;
                for (int i = 0; i < (int)D_len; i++)
                {
                    double element = BitConverter.ToDouble(serialized, curIndex);
                    curIndex += BitConverter.GetBytes(element).Length;
                    D.Add(element);
                }
                for (int i = 0; i < (int)K.Length; i++)
                {
                    K[i] = BitConverter.ToDouble(serialized, curIndex);
                    curIndex += BitConverter.GetBytes(K[i]).Length;
                }
                for (int i = 0; i < (int)R.Length; i++)
                {
                    R[i] = BitConverter.ToDouble(serialized, curIndex);
                    curIndex += BitConverter.GetBytes(R[i]).Length;
                }
                for (int i = 0; i < (int)P.Length; i++)
                {
                    P[i] = BitConverter.ToDouble(serialized, curIndex);
                    curIndex += BitConverter.GetBytes(P[i]).Length;
                }
                binning_x = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(binning_x).Length;
                binning_y = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(binning_y).Length;
                curIndex += roi.Deserialize(serialized, curIndex);
                return (curIndex - startIndex);
            }

        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinectIPSettings : MonoBehaviour {

    public List<string> ipAddresses = new List<string>();

    private void Awake()
    {
        ipAddresses.Add("");
        ipAddresses.Add("");
        ipAddresses.Add("");
    }

    public void Field1Edited(string ip)
    {
        ipAddresses[0] = ip;
    }

    public void Field2Edited(string ip)
    {
        ipAddresses[1] = ip;
    }

    public void Field3Edited(string ip)
    {
        ipAddresses[2] = ip;
    }

    public void Field4Edited(string ip)
    {
        ipAddresses[3] = ip;
    }

    public void Field5Edited(string ip)
    {
        ipAddresses[4] =  ip;
    }

    
}

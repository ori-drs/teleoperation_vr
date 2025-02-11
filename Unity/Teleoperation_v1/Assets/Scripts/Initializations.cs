using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
public class Initializations : MonoBehaviour
{
    public bool debugLog = false;
    
    // Start is called before the first frame update
    void Start()
    {
        Application.runInBackground = true;
    }

    // Update is called once per frame
    void Update()
    {
        if (!Application.runInBackground)
        {
            Application.runInBackground = true;

            if (debugLog)
            {
                Debug.Log("Re-Setting Application.runInBackground to TRUE at: " + Time.time);
            }
        }

    }
}

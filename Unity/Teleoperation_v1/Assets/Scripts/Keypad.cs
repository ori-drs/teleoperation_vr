using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;  // Add this for UI components handling
using TMPro;
public class Keypad : MonoBehaviour
{
    public TMP_InputField IPInputField;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void b1()
    {
        IPInputField.text = IPInputField.text + "1";
    }

    public void b2()
    {
        IPInputField.text = IPInputField.text + "2";
    }

    public void b3()
    {
        IPInputField.text = IPInputField.text + "3";
    }

    public void b4()
    {
        IPInputField.text = IPInputField.text + "4";
    }

    public void b5()
    {
        IPInputField.text = IPInputField.text + "5";
    }

    public void b6()
    {
        IPInputField.text = IPInputField.text + "6";
    }

    public void b7()
    {
        IPInputField.text = IPInputField.text + "7";
    }

    public void b8()
    {
        IPInputField.text = IPInputField.text + "8";
    }

    public void b9()
    {
        IPInputField.text = IPInputField.text + "9";
    }

    public void b0()
    {
        IPInputField.text = IPInputField.text + "0";
    }

    public void bdot()
    {
        IPInputField.text = IPInputField.text + ".";
    }

    public void clear()
    {
        // IPInputField.text = "";
        IPInputField.text = IPInputField.text.Substring(0, IPInputField.text.Length - 1);
    }
    
}

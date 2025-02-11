using UnityEngine;
using UnityEngine.InputSystem; // This is necessary for InputAction.CallbackContext
using System.Net;
using System.Net.Sockets;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.UI;  // Add this for UI components handling
using TMPro;
using System.Text.RegularExpressions;
using System.Linq;
using System.Collections.Generic;

public class ControllerInputSender_old : MonoBehaviour
{
    // public ThumbstickWatcher ThumbsWatcher;
    public ActionBasedController rightController; // Assign this in the Unity Inspector
    private UdpClient udpClient;
    private string ipAddress = "127.0.0.1";
    public int port = 7777;
    private bool isSending = false;
    public Button sendButton;  // Assign this button in the Unity Inspector
    public TMP_InputField IPInputField;  // Drag the InputField component to this field in the Unity Inspector
    public TMP_Text statusText;  // Assign this in the Unity Inspector
    private UnityEngine.XR.InputDevice device;
    void Start()
    {
        var rightHandDevices = new List<UnityEngine.XR.InputDevice>();
        // UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.LeftHand, leftHandDevices);
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.RightHand, rightHandDevices);

        if(rightHandDevices.Count == 1)
        {
            device = rightHandDevices[0];
            // Debug.Log(string.Format("Device name '{0}' with role '{1}'", device.name, device.role.ToString()));
        }
        // else if(leftHandDevices.Count > 1)
        // {
        //     Debug.Log("Found more than one left hand!");
        // }
        // ThumbsWatcher.primaryButtonPress.AddListener(onPrimaryButtonEvent);
        // IPInputField.ActivateInputField();
        ipAddress = IPInputField.text;
        // Ensure the controller is properly assigned
        if (rightController == null)
        {
            Debug.LogError("ActionBasedController is not assigned on ControllerInputSender.");
            UpdateStatus("Cannot find right controller");
        }
        else
        {
            UpdateStatus("Right controller connected");
            // Subscribe to input actions
            rightController.selectAction.action.started += OnGripPressed;
            rightController.selectAction.action.canceled += OnGripReleased;
            rightController.activateAction.action.performed += OnThumbstickMove;
        }
        // Initial button color
        if (sendButton != null)
            sendButton.GetComponent<Image>().color = Color.red;
            
    }

    public void ChangeStreamMode()
    {
        // IPInputField.ActivateInputField();
        // IPInputField.Select();
        Debug.Log(isSending ? "Streaming data via UDP" : "Stopped streaming data"); 
        ipAddress = IPInputField.text;
        if (IsValidIPAddress(ipAddress))
        {
            isSending = !isSending;
            Connect();
            // Update button color based on isSending
            if (sendButton != null)
            {
                sendButton.GetComponent<Image>().color = isSending ? Color.green : Color.red;
            }
        }
        else
        {
            UpdateStatus("Invalid IP address");
        }
    }

    void Update()
    {
        bool triggerValue;
        if (device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out triggerValue) && triggerValue)
        {
            Debug.Log("Trigger button is pressed.");
            UpdateStatus("Debug trigger button is pressed");
        }
        // if (device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxisClick, out triggerValue) && triggerValue)
        // {
        //     Debug.Log("Trigger button is pressed.");
        //     UpdateStatus("Debug primary2DAxisClick is pressed");
        // }
        
        if (device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out Vector2 tposition)) {
            Debug.Log("Joystick : " + tposition.x + "," + tposition.y);
            UpdateStatus("Joystick : " + tposition.x + "," + tposition.y);
        }  

        
        if (isSending)
        {
            Vector3 position = rightController.transform.position;
            Quaternion rotation = rightController.transform.rotation;
            SendData($"EE Position: [{position.x}, {position.y}, {position.z}]");
            SendData($"EE Orientation: [{rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}]");
            // UpdateStatus($"Streaming right controller data to IP {ipAddress}");
        }
        else
        {
            if (!IsValidIPAddress(ipAddress))
                UpdateStatus("Invalid IP address");
            else
                UpdateStatus("Stopped streaming data");
        }
    }

    void Connect()
    {
        udpClient = new UdpClient();
        udpClient.Connect(IPAddress.Parse(ipAddress), port);
    }
    
    void UpdateStatus(string message)
    {
        if (statusText != null)
            statusText.text = message;
    }
    
    private void OnGripPressed(InputAction.CallbackContext context)
    {
        SendData("EE Gripper: 1");
        UpdateStatus("grip changed");
    }

    private void OnGripReleased(InputAction.CallbackContext context)
    {
        SendData("EE Gripper: 0");
    }

    private void OnThumbstickMove(InputAction.CallbackContext context)
    {
        Vector2 thumbstickPosition = context.ReadValue<Vector2>();
        Debug.Log($"******** Thumbstick Position: {thumbstickPosition.x}, {thumbstickPosition.y}");
        SendData($"Thumbstick Position: {thumbstickPosition.x}, {thumbstickPosition.y}");
        UpdateStatus("thumbstick position changed");
    }

    void SendData(string message)
    {
        if (udpClient != null && isSending)
        {
            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(message);
            udpClient.Send(bytes, bytes.Length);
            Debug.Log($"Sent: {message}");  // Optional: Log what's being sent
        }
    }

    private void OnDestroy()
    {
        if (udpClient != null)
        {
            udpClient.Close();
        }

        // Unsubscribe from input actions
        if (rightController != null)
        {
            rightController.selectAction.action.started -= OnGripPressed;
            rightController.selectAction.action.canceled -= OnGripReleased;
            rightController.activateAction.action.performed -= OnThumbstickMove;
        }
    }
    
    private bool IsValidIPAddress(string ipAddr)
    {
        // Regex pattern for a basic IP address validation
        // string pattern = @"^(\d{1,3}\.){3}\d{1,3}$";
        // if (Regex.IsMatch(ipAddr, pattern))
        // {
        //     string[] segments = ipAddr.Split('.');
        //     return segments.All(s => int.TryParse(s, out int num) && num >= 0 && num <= 255);
        // }
        // return false;
        
        // Enhanced regex pattern for a strict IP address validation
        string pattern = @"^([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$";
        return Regex.IsMatch(ipAddress, pattern);
    
    }
}

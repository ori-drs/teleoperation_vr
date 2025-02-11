using UnityEngine;
using UnityEngine.InputSystem; // This is necessary for InputAction.CallbackContext
using System.Net;
using System.Net.Sockets;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.UI;  // Add this for UI components handling
using TMPro;
using System.Text.RegularExpressions;
using System.Collections.Generic;

public class ControllerInputSender : MonoBehaviour
{
    public ActionBasedController leftController; // Assign this in the Unity Inspector
    public ActionBasedController rightController; // Assign this in the Unity Inspector
    public Button sendButton;  // Assign this button in the Unity Inspector
    public TMP_InputField ipInputField;  // Drag the InputField component to this field in the Unity Inspector
    public TMP_Text statusText;  // Assign this in the Unity Inspector
    private UnityEngine.XR.InputDevice _leftThumbstick;
    private UnityEngine.XR.InputDevice _rightThumbstick;
    private UdpClient _udpClient;
    private string _ipAddress = "127.0.0.1";
    private int _port = 7777;
    private bool _isSending = false;
    
    void Start()
    {
        // Thumbstick uses a different interface
        var leftHandDevices = new List<UnityEngine.XR.InputDevice>();
        var rightHandDevices = new List<UnityEngine.XR.InputDevice>();
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.LeftHand, leftHandDevices);
        UnityEngine.XR.InputDevices.GetDevicesAtXRNode(UnityEngine.XR.XRNode.RightHand, rightHandDevices);

        _leftThumbstick = leftHandDevices[0];
        _rightThumbstick = rightHandDevices[0];
        
        _ipAddress = ipInputField.text;
        // Ensure the controller is properly assigned
        if (rightController == null && leftController == null)
        {
            Debug.LogError("ActionBasedController is not assigned on ControllerInputSender.");
            UpdateStatus("Cannot find controllers!");
        }
        else
        {
            UpdateStatus("Controllers connected");
            // Subscribe to input actions
            leftController.selectAction.action.started += OnLeftGripPressed;
            leftController.selectAction.action.canceled += OnLeftGripReleased;
            rightController.selectAction.action.started += OnRightGripPressed;
            rightController.selectAction.action.canceled += OnRightGripReleased;
            // rightController.activateAction.action.performed += OnThumbstickMove;
        }
        // Initial button color
        if (sendButton != null)
            sendButton.GetComponent<Image>().color = Color.green;
    }

    public void ChangeStreamMode()
    {

        Debug.Log(_isSending ? "Streaming data via UDP" : "Stopped streaming data"); 
        _ipAddress = ipInputField.text;
        if (IsValidIPAddress(_ipAddress))
        {
            _isSending = !_isSending;
            Connect();
            // Update button color based on _isSending
            if (sendButton != null)
            {
                sendButton.GetComponent<Image>().color = _isSending ? Color.red : Color.green;
            }
        }
        else
        {
            UpdateStatus("Invalid IP address");
        }
    }

    void Update()
    {
        
        if (_isSending)
        {
            if (_leftThumbstick.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out Vector2 leftthumbposition)) {
                SendData($"Left controller thumbstick: [{leftthumbposition.x}, {leftthumbposition.y}]");
            }  
            Vector3 leftposition = leftController.transform.position;
            Quaternion leftrotation = leftController.transform.rotation;
            SendData($"Left controller position: [{leftposition.x}, {leftposition.y}, {leftposition.z}]");
            SendData($"Left controller orientation: [{leftrotation.x}, {leftrotation.y}, {leftrotation.z}, {leftrotation.w}]");
            
            if (_rightThumbstick.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out Vector2 rightthumbposition)) {
                SendData($"Right controller thumbstick: [{rightthumbposition.x}, {rightthumbposition.y}]");
            }  
            Vector3 rightposition = rightController.transform.position;
            Quaternion rightrotation = rightController.transform.rotation;
            SendData($"Right controller position: [{rightposition.x}, {rightposition.y}, {rightposition.z}]");
            SendData($"Right controller orientation: [{rightrotation.x}, {rightrotation.y}, {rightrotation.z}, {rightrotation.w}]");
            
            UpdateStatus($"Streaming controllers data to IP {_ipAddress}");
        }
        else
        {
            if (!IsValidIPAddress(_ipAddress))
                UpdateStatus("Invalid IP address");
            else
                UpdateStatus("Stopped streaming data");
        }
    }

    void Connect()
    {
        _udpClient = new UdpClient();
        _udpClient.Connect(IPAddress.Parse(_ipAddress), _port);
    }
    
    void UpdateStatus(string message)
    {
        if (statusText != null)
            statusText.text = message;
    }
    
    private void OnLeftGripPressed(InputAction.CallbackContext context)
    {
        SendData("Left controller gripper: 1");
    }

    private void OnLeftGripReleased(InputAction.CallbackContext context)
    {
        SendData("Left controller gripper: 0");
    }
    
    private void OnRightGripPressed(InputAction.CallbackContext context)
    {
        SendData("Right controller gripper: 1");
    }

    private void OnRightGripReleased(InputAction.CallbackContext context)
    {
        SendData("Right controller gripper: 0");
    }
    
    private void SendData(string message)
    {
        if (_udpClient != null && _isSending)
        {
            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(message);
            _udpClient.Send(bytes, bytes.Length);
            Debug.Log($"Sent: {message}");  // Optional: Log what's being sent
        }
    }

    private void OnDestroy()
    {
        if (_udpClient != null)
        {
            _udpClient.Close();
        }

        // Unsubscribe from input actions
        if (rightController != null)
        {
            leftController.selectAction.action.started -= OnLeftGripPressed;
            leftController.selectAction.action.canceled -= OnLeftGripReleased;
            rightController.selectAction.action.started -= OnRightGripPressed;
            rightController.selectAction.action.canceled -= OnRightGripReleased;

        }
    }
    
    private bool IsValidIPAddress(string ipAddr)
    {
        string pattern = @"^([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$";
        return Regex.IsMatch(ipAddr, pattern);
    
    }
}

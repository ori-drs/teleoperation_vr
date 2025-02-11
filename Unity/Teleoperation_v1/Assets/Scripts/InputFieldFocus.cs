using UnityEngine;
using TMPro;
using UnityEngine.EventSystems; // Required for Event data

public class InputFieldFocus : MonoBehaviour, IPointerClickHandler
{
    public TMP_InputField inputField;

    void Start()
    {
        inputField.ActivateInputField();
        inputField.Select();
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        // Force the input field to gain focus
        inputField.ActivateInputField();
        inputField.Select();
    }
}
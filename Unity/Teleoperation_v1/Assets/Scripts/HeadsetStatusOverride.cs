using UnityEngine;
using System.Collections;

public class HeadsetStatusOverride : MonoBehaviour
{
    // Event or delegate to hook into the real headset status check
    public delegate void HeadsetStatusChanged(bool isWorn);
    public static event HeadsetStatusChanged OnHeadsetStatusChanged;

    void Start()
    {
        // Subscribe to the real headset status event
        if (OnHeadsetStatusChanged != null)
        {
            OnHeadsetStatusChanged += HandleHeadsetStatusChanged;
        }

        // Start a coroutine to continuously report headset is worn
        StartCoroutine(ReportHeadsetNotRemoved());
    }

    private void HandleHeadsetStatusChanged(bool isWorn)
    {
        // Normally handle the event
        Debug.Log("Real headset status: " + (isWorn ? "Worn" : "Not Worn"));

        // Optionally, stop the coroutine if the real headset is worn
        // StopCoroutine(ReportHeadsetNotRemoved());
    }

    IEnumerator ReportHeadsetNotRemoved()
    {
        while (true)
        {
            // Force the value to always report not removed
            OnHeadsetStatusChanged?.Invoke(true);

            // Wait for a second before reporting again
            yield return new WaitForSeconds(1);
        }
    }

    void OnDestroy()
    {
        // Unsubscribe to avoid memory leaks
        if (OnHeadsetStatusChanged != null)
        {
            OnHeadsetStatusChanged -= HandleHeadsetStatusChanged;
        }
    }
}
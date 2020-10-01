/**
 * @description: This file contains the code for graphing the accelerometer readings via plotly
 * @author: Marques Reimann
 * 9/16/2020
 */

// Set variables used for API call
var acc_dev_num = '25B702F0-A84D-A587-3FD1-AD183B22B549'
var acc_cha_num = 'F364140100B04240BA5005CA45BF8ABC'
var myHeaders = new Headers()
myHeaders.append(
  'Authorization',
  'Bearer 15c1fcab3a922c533814ea5d4a78e848dd9ba624',
)
// Request Options object for fetching the characteristics
var requestOptions = {
  method: 'GET',
  headers: myHeaders,
  redirect: 'follow',
}
// Request Options for triggering a rediscovering of the BLE device state
var startDiscoveryRequestOptions = {
  method: 'POST',
  headers: myHeaders,
  redirect: 'follow',
}

// Traces for the 3 accelerometer readings which will be graphed
var trace1 = {
  name: 'x-acceleration',
  y: [0],
  type: 'line',
}
var trace2 = {
  name: 'y-acceleration',
  y: [1],
  type: 'line',
}
var trace3 = {
  name: 'z-acceleration',
  y: [2],
  type: 'line',
}

// Initialize a plot using the 3 traces
Plotly.newPlot(
  'chart',
  [trace1, trace2, trace3],
  { margin: { t: 0, b: 0 } },
  { displayModeBar: false, responsive: true },
)

// Update the plot every 200 ms (5 times per second)
setInterval(initGetData, 200)

// Global var cnt used for appendData fcn
var cnt = 0

// Append a set of readings to the graph
function appendData(x_val, y_val, z_val) {
  Plotly.extendTraces('chart', { y: [[x_val], [y_val], [z_val]] }, [0, 1, 2])
  cnt++
  if (cnt > 10) {
    Plotly.relayout('chart', {
      xaxis: {
        range: [cnt - 10, cnt],
      },
    })
  }
}

// Global variable tcnt used for appendSimulatedData fcn
var tcnt = 0

// Append a set of readings to the graph
// This version adds a sin wave to simulate dynamic readings
function appendSimulatedData(x_val, y_val, z_val) {
  Plotly.extendTraces(
    'chart',
    {
      y: [
        [x_val + Math.sin(tcnt / Math.PI)],
        [y_val + Math.sin(tcnt / Math.PI)],
        [z_val + Math.sin(tcnt / Math.PI)],
      ],
    },
    [0, 1, 2],
  )
  tcnt++
  if (tcnt > 100) {
    Plotly.relayout('chart', {
      xaxis: {
        range: [tcnt - 100, tcnt],
      },
    })
  }
}

// Initialize a rediscovery of the BLE device state. This prevents the cached acceleration
//  value from being stale.
function initGetData() {
  fetch(
    `https://api.nrfcloud.com/v1/devices/${acc_dev_num}/discover`,
    startDiscoveryRequestOptions,
  )
    .then((response) => response.text())
    .then((result) => {
      getData()
    })
    .catch((error) => console.log('error', error))
}

// Function to make API call which gets accelerometer data
function getData() {
  fetch(
    `https://api.nrfcloud.com/v1/devices/${acc_dev_num}/characteristics/${acc_cha_num}`,
    requestOptions,
  )
    .then((response) => response.json())
    .then((result) => {
      console.log(result)

      // Check if there is a message, which means the characteristic is not connected.
      if (result.message) {
        console.log(message) // This will likely occur if rate is exceeded
        return
      } else {
        connected = true
        arr = decodeBytes(result)
        console.log('arr: ', arr)
        appendData(arr[0], arr[1], arr[2])
        //appendSimulatedData(arr[0], arr[1], arr[2]); // Simulated readings
      }
    })
    .catch((error) => console.log('error', error))
}

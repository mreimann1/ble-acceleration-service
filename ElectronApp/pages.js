// This file contains the code for landing page of the web app
const fetch = require('node-fetch')

// Set variables used for API call
var acc_dev_num = '25B702F0-A84D-A587-3FD1-AD183B22B549'
var acc_cha_num = 'F364140100B04240BA5005CA45BF8ABC'
var myHeaders = new Headers()
myHeaders.append(
  'Authorization',
  'Bearer 15c1fcab3a922c533814ea5d4a78e848dd9ba624',
)
var requestOptions = {
  method: 'GET',
  headers: myHeaders,
  redirect: 'follow',
}
var connected = false // the state of this devices connection to the acceleration device

function load_connect() {
  const text = /* html */ `  <div class="container">
    <div class="col align-items-center d-flex flex-column">
    <br>
    <div class="row-3 main-row">
    <img id="nextflex-logo" src="./img/logo_large.png" alt="nextflex logo">
    </div><br>
    <div class="row-3 main-row">
      <h1>Welcome to The Acceleration Service</h1>
    </div>
      <div>
        <button class="btn btn-warning btn-lg btn-block" type="button" id="check-conn-btn">Connect To Acceleration Service</button>
      </div><br>
    </div>
  </div>`
  document.getElementById('body').innerHTML = text
  console.log('Connect page loaded.')

  // Call check_connection method when abort btn clicked
  var abortButton = document.getElementById('check-conn-btn')
  abortButton.addEventListener('click', function () {
    console.log('Abort button clicked.')
    check_connection()
  })
}

load_connect()

// Function to make API call which checks if device is available.
function check_connection() {
  fetch(
    `https://api.nrfcloud.com/v1/devices/${acc_dev_num}/characteristics/${acc_cha_num}`,
    requestOptions,
  )
    .then((response) => response.json())
    .then((result) => {
      console.log(result)

      // Check if there is a message, which means the characteristic is not connected.
      if (result.message) {
        alert(result.message)
      } else {
        connected = true
        load_display() // load the display page.
      }
    })
    .catch((error) => console.log('error', error))
}

function load_display() {
  const text = /* html */ `  <div class="container">
      <div class="col align-items-center d-flex flex-column">
        <br>
        <div class="row-1 main-row">
            <img id="nextflex-logo" src="./img/logo_large.png" alt="nextflex logo">
        </div>
        <div class="status-field row-2">
            <button class="btn btn-warning btn-lg btn-block" type="button" id="get-rding-btn">Getting Readings</button>
            <iframe class="graph" src="./graph/plotlygraph.html"></iframe>
        </div>
        </div>`
  document.getElementById('body').innerHTML = text
  console.log('Connect page loaded.')

  // Call check_connection method when abort btn clicked
  var abortButton = document.getElementById('get-rding-btn')
  abortButton.addEventListener('click', function () {
    load_connect()
    console.log('Get readings button clicked.')
  })
}

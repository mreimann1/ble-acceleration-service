/***
 * @description: This javascript module serves as a decoder for the BLE custom Acceleration Service
 * @author: Marques Reimann
 * 9/14/2020
 */

// A example of the type of reading that will be sent over the service.
// Array x corresponds to a reading of {x: .1, y: .2, z: .4}
var x = [205, 204, 204, 61, 205, 204, 76, 62, 205, 204, 204, 62]

// This function decodes an array of bytes. Flips every consecutive 4-bytes
// @Argument inputArr: This is an array of integers which represent bytes
//                     The byte order composing each float is reversed, but
//                     the floats are in order.
// @Return:            An array of floats
function decodeBytes(inputArr) {
  var arrOfFloats = []
  for (var i = 0; i < inputArr.length; i += 4) {
    let index = 0
    let tempArr = inputArr.slice(i, i + 4)
    for (var j = 3; j >= 0; j--) {
      tempArr[index] = inputArr[i + j]
      index++
    }
    arrOfFloats.push(printFloat(tempArr, 0, 4))
  }
  return arrOfFloats
}
console.log('arrayOfFloats: ', decodeBytes(x))

// The following code integrated from stackoverflow
// This function returns the javascript standard float value of
// the bytes in arr between given indexes.
function printFloat(arr, start, stop) {
  var data = arr.slice(start, stop)

  var buf = new ArrayBuffer(4)
  var view = new DataView(buf)

  data.forEach(function (b, i) {
    view.setUint8(i, b)
  })
  var num = view.getFloat32(0)
  return num
}

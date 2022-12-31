var host = '';
if (window.location.hostname == 'localhost') {
  host = 'http://172.20.10.2/';
}

function setMode(mode) {
  $.get( host + "setMode?mode=" + mode, function() {
    //alert( "success" );
  });
}

function setLegsRaised(v) {
  $.get( host + "setGaitLegsRaised?raised=" + v, function() {
    //alert( "success" );
  });
}


function updateTagInfo() {
  $.getJSON( host + "tag", function( data ) {
    var config = data;

    $('#tagInfo').html(JSON.stringify(data));

    setTimeout(updateTagInfo, 200);
  });
}

function objToTable(obj) {
  var s = '<table class="dataTable"><thead>'+
    '<tr> <th scope="col">Param</th> <th scope="col">Value</th> </tr>'+
    '</thead> <tbody>';

  Object.keys(obj).forEach(function (item) {
    s += '<tr>';
    s += '<td><b>'+item+'</b></td>';
    if (typeof obj[item] === 'boolean') {
      s += '<td>'+(obj[item] ? '<button type="button" class="btn btn-sm btn-success">T</button>' : '<button type="button" class="btn btn-sm btn-danger">F</button>') +'</td>';
    } else {
      if (item == 'stepUrgency') {
        s += '<td>';
        if (obj[item] > 1) {
          s += '<div style="color:red; font-weight:bold;">'
        } else {
          s += '<div>';
        }
        s += + obj[item] + '</div></td>';
      } else
        s += '<td>'+obj[item]+'</td>';
    }
    s += '</tr>';
  });


  s += '</tbody> </table>';
  return s;
}

function updateStatus() {
  $.getJSON( host + "status", function( data ) {
    var config = data;

    var s = '<h3>Status</h3>';
    s += objToTable(data);
    $('#status').html(s);



    setTimeout(updateStatus, 200);
  });
}


function updateGait() {
  $.getJSON( host + "gait", function( data ) {
    var config = data;

    // prep a table for each leg
    var s = '';
    for (var i=5; i>2; i--) {
      s += '<h3>'+i+'</h3>';
      s += objToTable(data.legs[i]);
    }

    $('#gait1').html(s);

    // prep a table for each leg
    s = '';
    for (var i=0; i<3; i++) {
      s += '<h3>'+i+'</h3>';
      s += objToTable(data.legs[i]);
    }

    $('#gait2').html(s);

    setTimeout(updateGait, 200);
  });
}


$(function() {

  setTimeout(updateStatus, 500);

  setTimeout(updateGait, 200);

  // event handlers for mode buttons

  $('#passiveBut').click(()=>{
    setMode(0);
  });

  $('#safeBut').click(()=>{
    setMode(1);
  });

  $('#standBut').click(()=>{
    setMode(2);
  });

  $('#walkBut').click(()=>{
    setMode(3);
  });

  // gait legs raised
  $('#oneBut').click(()=>{
    setLegsRaised(1);
  });

  $('#twoBut').click(()=>{
    setLegsRaised(2);
  });

  $('#threeBut').click(()=>{
    setLegsRaised(3);
  });

});

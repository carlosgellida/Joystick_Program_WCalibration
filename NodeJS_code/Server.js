const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', function connection(ws) {
  
  ws.on('message', function incoming(message) {
    //console.log('received: %s', message);
    ws.send(message);
  });

  ws.on('message', function(message) {
    wss.broadcast(message);
 }); 
 
});

wss.broadcast = function broadcast(msg) {
  //console.log(msg);
  wss.clients.forEach(function each(client) {
      client.send(msg);
   });
};
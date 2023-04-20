WiFiClient wifi;
HttpClient client = HttpClient(wifi, localhost, port);

void postDataTowebServer(String rssi, String address)
{
  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, localhost, port);
  String payload = "rssi=" + rssi + "&adress=" + address;

  if (!client.connect(localhost, port))
  {
    Serial.println("Connection failed");
    return;
  }
  else
  {
    client.print(String(filedirectory) +
                 "Host: " + localhost + "\r\n" +
                 "Content-Type: application/x-www-form-urlencoded\r\n" +
                 "Content-Length: " + payload.length() + "\r\n" +
                 "Connection: close\r\n\r\n" +
                 payload);
    while (client.available() == 0)
    {
      delay(1);
    } 
    Serial.println("post data is completed");
    delay(1000);
  }

}
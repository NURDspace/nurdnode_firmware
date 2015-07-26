const int UADDRTRIP = 14;
const int MADDRTRIP = 15;
const int LADDRTRIP = 16;
const int TABLESIZE = 8;

struct addrlookup {
  int limit;
  int address;
};

addrlookup addrmap[TABLESIZE]
{
  {10, 0},
  {275, 4},
  {391, 2},
  {477, 6},
  {541, 1},
  {592, 5},
  {634, 3},
  {839, 7},
};


void setup() {
  Serial.begin(9600);
  pinMode(UADDRTRIP, INPUT);
  pinMode(MADDRTRIP, INPUT);
  pinMode(LADDRTRIP, INPUT);
}

void loop() {
  printval("Address: ", get_address());
  printval("H: ", analogRead(UADDRTRIP));
  printval("M: ", analogRead(MADDRTRIP));
  printval("L: ", analogRead(LADDRTRIP));
}

unsigned int get_address()
{
    unsigned int address = 0;
    address += (binread_triplet(UADDRTRIP) & 0x7) << 6;
    address += (binread_triplet(MADDRTRIP) & 0x7) << 3;
    address += (binread_triplet(LADDRTRIP) & 0x7);
    
    return address;
}

int binread_triplet(int pin)
{
    int analog_value = analogRead(pin);
    for (int i=0;i<TABLESIZE;i++) {
      if (analog_value < addrmap[i].limit) return addrmap[i].address;
    }
}


void printval(char* label, int  value)
{
    char buf[4];
    itoa(value, buf, 10);
    Serial.write(label);
    Serial.write(buf);
    Serial.write("\n");
}

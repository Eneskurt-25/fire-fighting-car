const int IN1=2,  IN2=3,  IN3=4,  IN4=5;
const int ENA=10, ENB=11;
const int TRIG=6, ECHO=7;
const int POMPA_PIN = 9;
const int ALEV_PIN  = A5;
const int S1=A0, S2=A1, S3=A2, S4=A3, S5=A4;

// --- HIZ AYARLARI ---
const int HIZ_DUZGIT   = 60;
const int HIZ_DON      = 90;
const int HIZ_SERT     = 130;
const int HIZ_GERI     = 60;
const int HIZ_KOR_DONUS = 120;
const int HIZ_ARAMA     = 90;

// --- ZAMANLAMALAR (ms) ---
const long T_ENGEL_GER          = 380;
const long T_ENGEL_DON          = 280;
const long T_POMPA              = 4000;
const long T_KESISIM_ILERI      = 200;
const long DONUS_SURESI_KOR_180    = 600;
const long DONUS_SURESI_KOR_KAVSAK = 250;
const long T_SONDUR_GERI        = 500;  // Söndürme sonrası geri gidiş süresi

const int  ALEV_ESIK   = 300;
const int  ENGEL_CM    = 6;

// --- KÖŞE / ÇİZGİ SONU EŞİKLERİ ---
const long T_ODA_SONU      = 280;   // 150 → 280ms (gürültü toleransı artırıldı)
const long T_VIRAJ_TOLERANS = 1500;

// --- DURUM MAKİNESİ ---
enum Durum {
  TAKIP_ET,
  KAVSAK_DONUS,
  SONDUR,
  SONDUR_GERI,   // YENİ: söndürme sonrası çizgiye geri dönüş
  DONUS_180,
  ENGEL_GERI,
  ENGEL_DON,
  TAMAMLANDI
};
Durum durum = TAKIP_ET;

unsigned long durumZamanlayici = 0;
unsigned long cizgiKayipZaman  = 0;
bool aleviSondurdu = false;
int  sonHata       = 0;
int  donusNiyeti   = 0;

int  sMesafe  = 999;
bool sAtes    = false;
bool sCizgiYok = false;
int  s1_val, s2_val, s3_val, s4_val, s5_val;

// ================================================================
void setup() {
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  pinMode(POMPA_PIN,OUTPUT); digitalWrite(POMPA_PIN,LOW);
  pinMode(S1,INPUT); pinMode(S2,INPUT); pinMode(S3,INPUT);
  pinMode(S4,INPUT); pinMode(S5,INPUT);
  pinMode(ALEV_PIN,INPUT);

  Serial.begin(9600);
  Serial.println("5 sn bekleniyor...");
  delay(5000);
  Serial.println("Basladi!");
}

// ================================================================
void loop() {
  unsigned long suAn = millis();

  sMesafe = mesafeOlc();
  sAtes   = (analogRead(ALEV_PIN) < ALEV_ESIK);
  s1_val  = digitalRead(S1); s2_val = digitalRead(S2); s3_val = digitalRead(S3);
  s4_val  = digitalRead(S4); s5_val = digitalRead(S5);

  sCizgiYok = (s1_val==HIGH && s2_val==HIGH && s3_val==HIGH && s4_val==HIGH && s5_val==HIGH);

  // KAVŞAK TESPİTİ — S3 merkez şartı eklendi (virajda yanlış tetiklenme önlendi)
  bool sagAlgilandi = (s4_val == LOW && s5_val == LOW && s3_val == LOW);
  bool solAlgilandi = (s1_val == LOW && s2_val == LOW && s3_val == LOW);
  bool kavsakVar    = (sagAlgilandi || solAlgilandi);

  switch (durum) {

    // ------------------------------------------------------------
    case TAKIP_ET:
      if (sMesafe > 0 && sMesafe < ENGEL_CM) {
        robotDur();
        durumZamanlayici = suAn;
        durum = ENGEL_GERI;
      }
      else if (sAtes && !aleviSondurdu) {
        robotDur();
        durumZamanlayici = suAn;
        durum = SONDUR;
      }
      else if (kavsakVar) {
        robotDur();
        delay(30);

        // Teyit okuma
        bool sagTeyit = (digitalRead(S4)==LOW || digitalRead(S5)==LOW) && digitalRead(S3)==LOW;
        bool solTeyit = (digitalRead(S1)==LOW || digitalRead(S2)==LOW) && digitalRead(S3)==LOW;

        if (sagTeyit) {
          donusNiyeti      = 1;
          cizgiKayipZaman  = 0;
          durum            = KAVSAK_DONUS;
        }
        else if (solTeyit) {
          donusNiyeti      = -1;
          cizgiKayipZaman  = 0;
          durum            = KAVSAK_DONUS;
        }
        else {
          durum = TAKIP_ET;
        }
      }
      else if (sCizgiYok) {
        if (cizgiKayipZaman == 0) cizgiKayipZaman = suAn;

        cizgiIzle();

        if (sonHata == 0) {
          if (suAn - cizgiKayipZaman >= T_ODA_SONU) {
            robotDur();
            cizgiKayipZaman = 0;
            sonHata = 0;
            if (sAtes && !aleviSondurdu) {
              durumZamanlayici = suAn;
              durum = SONDUR;
            } else {
              durum = DONUS_180;
            }
          }
        }
        else {
          if (suAn - cizgiKayipZaman >= T_VIRAJ_TOLERANS) {
            robotDur();
            cizgiKayipZaman = 0;
            sonHata = 0;
            durum = DONUS_180;
          }
        }
      }
      else {
        cizgiKayipZaman = 0;
        cizgiIzle();
      }
      break;

    // ------------------------------------------------------------
    case KAVSAK_DONUS:
      if (donusNiyeti == 1) merkezleyerekSagDon();
      else                  merkezleyerekSolDon();
      cizgiKayipZaman = 0;
      sonHata         = 0;
      durum           = TAKIP_ET;
      break;

    // ------------------------------------------------------------
    case SONDUR:
      digitalWrite(POMPA_PIN, HIGH);
      if (suAn - durumZamanlayici >= T_POMPA) {
        digitalWrite(POMPA_PIN, LOW);
        aleviSondurdu    = true;
        durumZamanlayici = suAn;
        durum            = SONDUR_GERI; // Söndürme bitti → geri çekil
      }
      break;

    // ------------------------------------------------------------
    // YENİ DURUM: Söndürme sonrası geri gidip çizgiyi ara
    case SONDUR_GERI:
      geriGit(HIZ_GERI);
      if (suAn - durumZamanlayici >= T_SONDUR_GERI) {
        robotDur();
        cizgiKayipZaman = 0;
        sonHata         = 0;
        durum           = DONUS_180;
      }
      break;

    // ------------------------------------------------------------
    case DONUS_180:
      merkezleyerek180Don();
      cizgiKayipZaman = 0;
      sonHata         = 0;
      durum           = TAKIP_ET;
      break;

    // ------------------------------------------------------------
    case ENGEL_GERI:
      geriGit(HIZ_GERI);
      if (suAn - durumZamanlayici >= T_ENGEL_GER) {
        robotDur();
        durumZamanlayici = suAn;
        durum = ENGEL_DON;
      }
      break;

    // ------------------------------------------------------------
    case ENGEL_DON:
      tamEksenSagDon(HIZ_KOR_DONUS);
      if (suAn - durumZamanlayici >= T_ENGEL_DON) {
        robotDur();
        durum = TAKIP_ET;
      }
      break;

    // ------------------------------------------------------------
    case TAMAMLANDI:
      robotDur();
      break;
  }
}

// ================================================================
//  KAVŞAK DÖNÜŞ FONKSİYONLARI
// ================================================================
void merkezleyerekSagDon() {
  robotDur();
  delay(200);

  ileriGit(HIZ_DUZGIT);
  delay(T_KESISIM_ILERI);
  robotDur();
  delay(100);

  tamEksenSagDon(HIZ_KOR_DONUS);
  delay(DONUS_SURESI_KOR_KAVSAK);

  tamEksenSagDon(HIZ_ARAMA);
  unsigned long baslangic = millis();
  while (digitalRead(S3) == HIGH && (millis() - baslangic < 2000)) {}

  robotDur();
  delay(200);
}

void merkezleyerekSolDon() {
  robotDur();
  delay(200);

  ileriGit(HIZ_DUZGIT);
  delay(T_KESISIM_ILERI);
  robotDur();
  delay(100);

  tamEksenSolDon(HIZ_KOR_DONUS);
  delay(DONUS_SURESI_KOR_KAVSAK);

  tamEksenSolDon(HIZ_ARAMA);
  unsigned long baslangic = millis();
  while (digitalRead(S3) == HIGH && (millis() - baslangic < 2000)) {}

  robotDur();
  delay(200);
}

// ================================================================
//  180 DERECE TANK DÖNÜŞÜ
// ================================================================
void merkezleyerek180Don() {
  robotDur();
  delay(250);

  tamEksenSolDon(HIZ_KOR_DONUS);
  delay(DONUS_SURESI_KOR_180);

  tamEksenSolDon(HIZ_ARAMA);
  unsigned long baslangic = millis();
  while (digitalRead(S3) == HIGH && (millis() - baslangic < 3000)) {}

  robotDur();
  delay(200);
}

// ================================================================
//  DİFERANSİYEL ÇİZGİ TAKİP MANTIĞI
// ================================================================
void cizgiIzle() {
  int  hata        = 0;
  bool cizgiGoruldu = true;

  if      (s3_val == LOW) hata =  0;
  else if (s2_val == LOW) hata = -1;
  else if (s4_val == LOW) hata =  1;
  else if (s1_val == LOW) hata = -2;
  else if (s5_val == LOW) hata =  2;
  else                    cizgiGoruldu = false;

  int solHiz = HIZ_DUZGIT;
  int sagHiz = HIZ_DUZGIT;

  if (cizgiGoruldu) {
    sonHata = hata;

    if      (hata == -2) { solHiz = 0;         sagHiz = HIZ_SERT; }
    else if (hata == -1) { solHiz = 0;         sagHiz = HIZ_DON;  }
    else if (hata ==  0) { solHiz = HIZ_DUZGIT; sagHiz = HIZ_DUZGIT; }
    else if (hata ==  1) { solHiz = HIZ_DON;   sagHiz = 0;        }
    else if (hata ==  2) { solHiz = HIZ_SERT;  sagHiz = 0;        }
  }
  else {
    // Çizgi kayboldu — son hataya göre arama yap
    if      (sonHata < 0) { solHiz = 0;              sagHiz = HIZ_DON; }
    else if (sonHata > 0) { solHiz = HIZ_DON;         sagHiz = 0;      }
    else                  { solHiz = HIZ_DUZGIT / 2; sagHiz = HIZ_DUZGIT / 2; } // DÜZELTME: donma yerine yavaş ileri
  }

  motorSur(solHiz, sagHiz);
}

// ================================================================
//  MOTOR SÜRÜCÜ VE YÖN FONKSİYONLARI
// ================================================================
void motorSur(int solHiz, int sagHiz) {
  if (solHiz >= 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    analogWrite(ENA, solHiz);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, -solHiz);
  }

  if (sagHiz >= 0) {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENB, sagHiz);
  } else {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, -sagHiz);
  }
}

int mesafeOlc() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long s = pulseIn(ECHO, HIGH, 10000);
  return (s == 0) ? 999 : (int)(s * 0.034 / 2);
}

void robotDur() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  analogWrite(ENA, 0);   analogWrite(ENB, 0);
}

void ileriGit(int v) {
  analogWrite(ENA, v);     analogWrite(ENB, v);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void geriGit(int v) {
  analogWrite(ENA, v);     analogWrite(ENB, v);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void solDon(int v) {
  analogWrite(ENA, 0);    analogWrite(ENB, v);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void sagDon(int v) {
  analogWrite(ENA, v);    analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void tamEksenSagDon(int hiz) {
  analogWrite(ENA, hiz);   analogWrite(ENB, hiz);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void tamEksenSolDon(int hiz) {
  analogWrite(ENA, hiz);   analogWrite(ENB, hiz);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

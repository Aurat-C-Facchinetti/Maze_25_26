# Maze_25_26
Code Repo for the Rescue Maze Project

# DA PYTHON AD ARDUINO

--- MOVIMETNO ---
- w --> muove in avanti. Prototipo metodo: **Prototipo metodo: invia(w030,);**
- Curve : diremo di quanti gradi curvare. Diremo A (per andare a sinistra) oppure D (per ruotare a destra); assieme alla direzione verranno indicati anche di quanti gradi deve essere la rotazione
verrò utilizzato un terzo parametro per retrocompatibiltà con codice già esistente. **Prototipo metodo: invia(a090,);**
- s--> switch fronte/retro **Prototipo metodo: invia(s,);**

--- FEATURES --- 
- h --> gestione harmed victim : lancio di due cubi e (forse) accensione led. **Prototipo metodo: invia(h0,);** dove 0 e 1 rappresentano destra e sinitra
- s --> gestione stable victim : lancio di un cubo e (forse) accensione led. **Prototipo metodo: invia(s0,);** dove 0 e 1 rappresentano destra e sinitra
- u --> gestione unharmed victim : (forse) accensione led. **Prototipo metodo: invia(u,);**

--- SENSORI ---
  - g --> giroscopio (in caso volessimo correggerci) (destra/sinistra) **Prototipo metodo: invia(g,);**
  - m + numero --> chiede ad arduino i dati di uno dei TOF (1-4) se dal lato dato c'è un muro e nel caso sia così ci facciamo ritornare un booleano/ 0-1 diseganre i muri **Prototipo metodo: invia(m1,);**
  - i --> inclinazione (fa parte di una funzione del movimento che appartenente al giroscopio) **Prototipo metodo: invia(i,);**
  - c --> colore **Prototipo metodo: invia(c,);**

# DA ARDUINO A PYTHON
  
---> FEEDBACK: <---
  - abbiamo bisogno di una conferma che ci sia stato un movimento **(vogliamo 1/0 per sapere se il movimento è avvenuto oppure no)**
  - ci facciamo passare in numero la distanza affraverso il tof che impostiamo a -1 se fallisce la misura (in caso sia troppo vicino o troppo lontano) **(vogliamo una distanza in millimetri)**
  - gradi del giroscopio sia per l'inclinazione **(vogliamo i gradi non i radianti)**
  - gradi del giroscopio per la rotazione (destra/sinistra) **(vogliamo i gradi non i radianti)**
  - conferma comando ricevuto per una sincronizzazione (restituisco il comando che ha ricevuto) **(voglaimo che ci risponda con l'esatto comando che abbiamo inviato)**
  - colore della piastrella preferibilemente un carattere **(vogliamo che ci restituisca un carattere)**
    

  


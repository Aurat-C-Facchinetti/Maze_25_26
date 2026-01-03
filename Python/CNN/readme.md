CNN - Convolutional Neural Network
Contenuto della cartella:

- Main.py : il file principale. Suddiviso in funzioni, questo file permette di costruire interamente una CNN, livello per livello. L'allenameto della rete avviene attravero immagini (che purtroppo non è stato possibile inserire direttamente in github poichè troppo pesanti). L'alberatura della cartella "Images" deve essere la seguente altrimenti non funziona: Images\A\immagine1.jpg.. Questo per ogni lettera (il dataset che è stato usato per allenare il modello è il seguente: https://www.kaggle.com/datasets/thomasqazwsxedc/alphabet-characters-fonts-dataset?resource=download-directory).
- Test.py : una volta creato ed allenato il modello è possibile utilizzare questa semplice interfaccia per testarne le bontà. Il modello allenato viene esportato in formato pth (fondamentale che sia così).
- letter_recognition_model_rotated.pth : è il risultato dell'allenamento della rete. In futuro sarà necessario rimuovere l'ultimo livello di qeusto modello per aggiungerne uno dedicato (con le foto che dovremo fare)
- label_encoder_rotated.npy : roba che serve al modello per funzionare corretametne (anch'esso generato durante la fase di finalizzazione allenamento). Converte variabili categoriche in numeri (non ci interessa al momento)
- MANCANTE : per far partire l'allenamento ovviamente serve il dataset linkato sopra: sarà sufficiente estrarre la cartella images all'interno della directory nella quale si trova il file main.py.

- Attenzione : il main è nato per sfruttare la GPU (nella mia configurazione una RTX 3060 12GB) è quindi molto importante, nella fase di setup del sistema, verificare di essere in possesso dei requisiti (supporto a CUDA 12.5), altrimenti il programma funzionerà il modalità CPU (molto lento). Qualora si riscontrassero problemi riguardo al funzionamento della GPU, bruh.

- Disclaimer : ho chiesto a deepseek di fare in modo che non facesse troppo schifo come codice, senza successo. Almeno è diviso in funzioni e c'è abbastanza debug da renderlo comprensibile... Non ho rimosso faccine, freccine, cose verdi, cose rosse ed emoji varie. In questo momento è importante capire il funzionamento teorico del sistema, non il codice (come sempre, al codice ci si arriva dopo, l'importante è avere chiaro in mente l'obiettivo)

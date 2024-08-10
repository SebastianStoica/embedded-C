#include <c8051F040.h>	// declaratii SFR
#include <uart1.h>
#include <Protocol.h>
#include <UserIO.h>

extern nod retea[];						// reteaua Master-Slave, cu 5 noduri

extern unsigned char TIP_NOD;			// tip nod
extern unsigned char ADR_MASTER;	// adresa nodului master

extern unsigned char timeout;		// variabila globala care indica expirare timp de asteptare eveniment
//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i);				// primire mesaj de la nodul i
unsigned char ascii2bin(unsigned char *ptr);			// functie de conversie 2 caractere ASCII HEX in binar

//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i){					// receptie mesaj															   
	unsigned char j, ch, sc, adresa_hw_dest, adresa_hw_src, screc, src, dest, lng, tipmes, *ptr;
	if(TIP_NOD == MASTER || TIP_NOD == JETON)														// Daca nodul este master (asteapta raspuns de la slave)
	{														// sau a transmis jetonul si asteapta confirmarea preluarii acestuia - un mesaj JET_MES
	   ch = UART1_Getch_TMO(WAIT);															// M: asteapta cu timeout primul caracter al raspunsului de la slave
		 if (timeout) return TMO;														// M: timeout, terminare receptie
		 retea[i].full = 0;														// M: raspunsul de la slave vine, considera ca mesajul anterior a fost transmis cu succes	
		 if (ch != ':')
		 {
		    do ch = UART1_Getch_TMO(5);
			while (timeout==0);			 // M: daca primul caracter nu este ':'...
			return ERI;
		 }
		ptr = retea[ADR_NOD].bufasc + 1;								// M: ignora restul mesajului
		*ptr++ = UART1_Getch_TMO(5);
		if(timeout) return CAN;	
		*ptr++ = UART1_Getch_TMO(5);
		if(timeout) return CAN;	
								// M: initializare pointer in bufferul ASCII
		adresa_hw_dest = ascii2bin(ptr);													
		if (adresa_hw_dest != ADR_NOD){
			do ch = UART1_Getch_TMO(5);
			while (timeout==0);			 // M: daca primul caracter nu este ':'...
			return ERA;												// M: asteapta cu timeout primul caracter ASCII al adresei HW
		}
	}
	else //pana aici e bun
	{
		do{
			do{
				ch = UART1_Getch_TMO(2*WAIT +ADR_NOD*WAIT);    // M: asteapta cu timeout al doilea caracter al adresei HW
				if(timeout) return TMO;														// M: timeout, terminare receptie
			}while(ch!=':');
			
			ptr = retea[ADR_NOD].bufasc + 1;														
			*ptr++ = UART1_Getch_TMO(5);														// M: daca mesajul nu este pentru acest nod	
			if (timeout)return CAN;															// M: ignora restul mesajului
			*ptr-- = UART1_Getch_TMO(5);														
			if (timeout)return CAN;																// M: adresa HW ASCII gresita, terminare receptie
			adresa_hw_dest = ascii2bin(ptr);
		}while(adresa_hw_dest != ADR_NOD);													
	}
	
	ptr++;
	do{													// Daca nodul este slave sau daca nu are jetonul ...
	*(++ptr) = UART1_Getch_TMO(5);
		if(timeout) return CAN;	
	}while(*ptr != 0x0A);
	
	ptr = retea[ADR_NOD].bufasc + 3;
	screc = adresa_hw_dest;													// S: asteapta cu timeout primirea primului caracter al unui mesaj (de la master sau de la cel care are jetonul)
	adresa_hw_src = ascii2bin(ptr);
	ptr+=2;													// (sau de la nodul care detine jetonul)
	screc += adresa_hw_src;	
	if(TIP_NOD == SLAVE)													// S: timeout, terminare receptie, nodul va deveni master
	{	
		ADR_MASTER = adresa_hw_src;													// sau va anunta ca s-a pierdut jetonul si va regenera jetonul
	}	
	tipmes = ascii2bin(ptr);													
	ptr+=2;														// S: asteapta sincronizarea cu inceputul mesajului
	screc += adresa_hw_src;	
	if(tipmes > 1)	return TIP;		
	screc += tipmes;														// S: initializeaza pointerul in bufferul ASCII
	if(tipmes == USER_MES)	
	{												// S: asteapta cu timeout primul caracter ASCII al adresei HW
		src = ascii2bin(ptr);	
		ptr += 2;											// S: timeout, terminare receptie
		screc += src;
		dest = ascii2bin(ptr);
		ptr += 2;											// S: asteapta cu timeout al doilea caracter al adresei HW
		screc += dest;											// S: timeout, terminare receptie
		if(TIP_NOD==MASTER)	 	
		  if(retea[dest].full == 1) return OVR;												// S: determina adresa HW destinatie
		lng = ascii2bin(ptr);	
		ptr += 2;										
		screc += lng;	
		if(TIP_NOD ==MASTER)						
		{										// M+S: initializeaza screc cu adresa HW dest
		    retea[dest].bufbin.adresa_hw_src = ADR_NOD;
			retea[dest].bufbin.tipmes = tipmes;
			retea[dest].bufbin.src = src;
			retea[dest].bufbin.dest = dest;					// M+S: determina adresa HW src
			retea[dest].bufbin.lng = lng;
				for (j = 0; j < lng; j++)									
				{
				  retea[dest].bufbin.date[j]  = ascii2bin(ptr);												
				  ptr++;												// M+S: cod functie eronat, terminare receptie
				  screc += retea[dest].bufbin.date[j] ;								// M+S: aduna adresa HW src
				}	  												// M+S: ia in calcul in screc codul functiei
			 sc = ascii2bin(ptr);
			if(sc == screc)												// M+S: Daca mesajul este USER_MES
			{												// M+S: determina sursa mesajului
			retea[dest].full = 1 ;
														
			return ROK;			//receive ok									// M+S: ia in calcul in screc adresa src
			}
				else return ESC;//err suma de control
			}												// M+S: determina destinatia mesajului
			else
			{													
				retea[ADR_NOD].bufbin.src = src;
				retea[ADR_NOD].bufbin.lng = lng;									// M+S: ia in calcul in screc adresa dest
				for (j = 0; j < lng; j++) {
					retea[ADR_NOD].bufbin.date[j] = ascii2bin(ptr);											// Daca nodul este master...
			 		ptr++;										// M: bufferul destinatie este deja plin, terminare receptie
					screc += retea[ADR_NOD].bufbin.date[j] ;
				}												// M+S: determina lng
			 sc = ascii2bin(ptr);
			 if(sc == screc)
			 {														
				retea[ADR_NOD].full = 1 ;											// M+S: ia in calcul in screc lungimea datelor
				return ROK;
			 }												// Daca nodul este master...
			 else return ESC;													// M: stocheaza in bufbin adresa HW src	egala cu ADR_NOD
		 }													// M: stocheaza in bufbin tipul mesajului	
	 }														// M: stocheaza in bufbin adresa nodului sursa al mesajului	
		else{																// M: stocheaza in bufbin adresa nodului destinatie al mesajului															// M: stocheaza lng
			retea[ADR_NOD].bufbin.adresa_hw_src = adresa_hw_src;
			sc = ascii2bin(ptr);
			if(sc == screc)	return POK;																
			else return ESC;
		}										// M: determina un octet de date
			
														// M+S: eroare SC, terminare receptie
																
//		return TMO;			// simuleaza asteptarea mesajului si iesirea cu timeout
}															


//***********************************************************************************************************
unsigned char ascii2bin(unsigned char *ptr){			// converteste doua caractere ASCII HEX de la adresa ptr
	unsigned char bin;
	
	if(*ptr > '9') bin = (*ptr++ - 'A' + 10) << 4;	// contributia primului caracter daca este litera
	else bin = (*ptr++ - '0') << 4;									// contributia primului caracter daca este cifra
	if(*ptr > '9') bin  += (*ptr++ - 'A' + 10);			// contributia celui de-al doilea caracter daca este litera
	else bin += (*ptr++ - '0');											// contributia celui de-al doilea caracter daca este cifra
	return bin;
}
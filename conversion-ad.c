/* 
 * Démontre comment utiliser la conversion analogique-numérique en
 * affichant la tension de sortie d'un potentiomètre sur un afficheur 
 * LED 7 segments
 * @author jean-michel-gonet
 */

#include <xc.h>
#include <stdio.h>

/**
 * Bits de configuration:
 */
#pragma config FOSC = INTIO67  // Oscillateur interne, ports A6 et A7 comme IO.
#pragma config IESO = OFF      // Pas d'embrouilles avec l'osc. au démarrage.
#pragma config FCMEN = OFF     // Pas de monitorage de l'oscillateur.

#pragma config MCLRE = INTMCLR // Pas de Master Reset. RE3 est un port IO.s

/**
 * Table de conversion pour afficheur LED 7 segments.
 */
unsigned char AFFICHAGE_led[] = {0x3f,0x06,0x5b,0x4f,
                                 0x66,0x6d,0x7d,0x07,
                                 0x7f,0x6f,0x77,0x7c,
                                 0x39,0x5e,0x79,0x71};
/**
 * Contenu à afficher sur l'afficheur 7 segments multiple.
 */
unsigned char AFFICHAGE_contenu[7];

/**
 * Prochain digit � activer.
 */
unsigned char AFFICHAGE_digit = 0;

/**
 * À chaque appel, affiche un digit sur l'afficheur
 * multiple. Cette fonction est à appeler successivement
 * pour obtenir l'affichage de tous les digits.
 */
void AFFICHAGE_raffraichir() {
    unsigned char a,c;

    c = 1 << AFFICHAGE_digit;
    a = AFFICHAGE_contenu[AFFICHAGE_digit] - '0';
    a = ~AFFICHAGE_led[a];

    PORTC = 0;
    PORTA = a;
    PORTC = c;

    if(AFFICHAGE_digit++>5) {
        AFFICHAGE_digit=0;
    }
}

/**
 * Lance une conversion A/D
 */
void AD_lanceConversion() {
    // Lance une conversion:
    ADCON0bits.GO = 1;
}

/**
 * Récupère le résultat de la conversion A/D
 * @return La valeur récupérée de ADRESH:ADRESL.
 */
unsigned int AD_termineConversion() {
    unsigned int ad = ADRESH;
    ad<<=8;
    ad+=ADRESL;
    return ad;
}

/**
 * Compte les interruptions du temporisateur, pour
 * mesurer les secondes.
 */
unsigned char tics=0;

/**
 * Routine des interruptions.
 * Détecte de quelle interruption il s'agit, puis appelle la fonction
 * correspondante.
 */
void interrupt interruptions() {
    if (INTCONbits.TMR0IF) {
        // Baisse le drapeau d'interruption pour la prochaine fois.
        INTCONbits.TMR0IF = 0;

        // Établit le compteur du temporisateur 0 pour un débordement
        // dans 6.67ms:
        TMR0H = 0xF9;
        TMR0L = 0x7D;
        AFFICHAGE_raffraichir();
        if (++tics >= 25) {
            tics=0;
            AD_lanceConversion();
        }
    }

    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        sprintf(AFFICHAGE_contenu,"%ud",AD_termineConversion());
    }
}

/**
 * Point d'entrée.
 * Configure les interruptions, le module A/D pour AN8, puis
 * va faire dodo.
 */
void main() {
    // Configure le port A et les 6 LSB du port C comme sorties digitales,
    // pour pouvoir contrôler l'afficheur.
    TRISA = 0;
    TRISC = 0;

    // Configure le module A/D:
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELBbits.ANSB2 = 1;

    ADCON0bits.ADON = 1;    // Allume le module A/D
    ADCON0bits.CHS = 8;     // Branche le convertisseur sur AN8
    ADCON2bits.ADFM = 1;    // Les 8 bits moins sign. de la conversion ...
                            // ... sont sur ADRESL
    ADCON2bits.ACQT = 3;    // Temps d'acquisition à 6 TAD.
    ADCON2bits.ADCS = 0;    // À 1MHz, le TAD dure 2us.

    // Configure le temporisateur 0 pour obtenir 150 interruptions par seconde,
    // en assumant que le microprocesseur est cadencé � 1MHz
    T0CONbits.TMR0ON = 1;  // Active le temporisateur 0.
    T0CONbits.T08BIT = 0;  // 16 bits pour compter jusqu'à 3125.
    T0CONbits.T0CS = 0;    // On utilise Fosc/4 comme source.
    T0CONbits.PSA = 1;     // On n'utilise pas le diviseur de fréquence.

    // Configure les interruptions
    RCONbits.IPEN = 1;      // Active le mode Haute / Basse priorité.
    INTCONbits.GIEH = 1;    // Active les interr. haute priorité.
    INTCONbits.GIEL = 0;    // Désactive les interr. basse priorité.

    INTCONbits.TMR0IE = 1;  // Active les interr. temporisateur 0
    PIE1bits.ADIE = 1;      // Active les interr. A/D
    IPR1bits.ADIP = 1;      // Interr. A/D sont de haute priorité.

    // Dodo (la routine d'interruptions fait tout le travail).
    while(1);
}

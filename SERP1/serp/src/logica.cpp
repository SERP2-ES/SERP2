#include "logica.h"
#include <ros/ros.h>
void verificarBlocos(int ligacoes[][100], float matrixValores[][100], float velocidades[2])
{
    float motorEsq;
    float motorDir;
    int antigoTemp1;
    int activateTemp1;
    int antigoTemp2;
    int activateTemp2;
    int aux;
    int i;
    int j;
    //Processamento de blocos
    bool blocoInv1;
    bool blocoInv2;
    bool blocoSoma1;
    bool blocoSoma2;
    bool blocoTemp1;
    bool blocoTemp2;
    bool blocoDecisor1;
    bool blocoDecisor2;
    bool blocoProduto;
    bool blocoIf;
    bool blocoElse;
    bool blocoAnd;
    bool blocoOr;
    bool blocoMaior;
    bool blocoMenor;
    bool blocoIgual;
    bool blocoTrans;
    int l;
    int k;

    //Verificacao das entradas
    bool entradaSoma11;
    bool entradaSoma12;
    bool entradaSoma21;
    bool entradaSoma22;
    bool entradaMux11;
    bool entradaMux12;
    bool entradaMux21;
    bool entradaMux22;
    bool entradaDec1;
    bool entradaDec2;
    bool entradaProd1;
    bool entradaProd2;
    bool entradaIf1;
    bool entradaIf2;
    bool entradaElseA;
    bool entradaElseB;
    bool entradaElseC;
    bool entradaElseD;
    bool entradaAnd1;
    bool entradaAnd2;
    bool entradaOr1;
    bool entradaOr2;
    bool entradaMaior1;
    bool entradaMaior2;
    bool entradaMenor1;
    bool entradaMenor2;
    bool entradaIgual1;
    bool entradaIgual2;
    bool entradaTrans1;

    int rc = 100;

    //----------------------------------------------------------------------
    //
    //
    //
    //Verifica entradas
    //
    //
    //
    //----------------------------------------------------------------------
    //Bloco Inversor 1 nao usado
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[6][i] == 1)
        {
            blocoInv1 = true;
        }
        i = i + 1;
    }

    //Bloco Inversor 2 nao usado
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[15][i] == 1)
        {
            blocoInv2 = true;
        }
        i = i + 1;
    }

    //Bloco Produto nao usado
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[20][i] == 1)
        {
            blocoProduto = true;
        }
        i = i + 1;
    }

    //bloco produto novo
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[20][i] == 1)
        {
            entradaProd1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[38][i] == 1)
        {
            entradaProd2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[21][i] == 1)
        {
            if ((entradaProd1 == true) && (entradaProd2 == true))
                blocoProduto = true;
        }
        i = i + 1;
    }
    //bloco if
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[39][i] == 1)
        {
            entradaIf1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[40][i] == 1)
        {
            entradaIf2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[41][i] == 1)
        {
            if ((entradaIf1 == true) && (entradaIf2 == true))
            {
                blocoIf = true;
                //printf("1\n");
            }
        }
        i = i + 1;
    }

    //Bloco Soma 1 nao usado
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[8][i] == 1)
        {
            entradaSoma11 = true;
            //ROS_INFO("ENTRADA1 CONFIRMADA");
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1 + 1)
    {
        if (ligacoes[32][i] == 1)
        {
            entradaSoma12 = true;
            //ROS_INFO("ENTRADA2 CONFIRMADA");
        }
        i = i + 1;
    }

    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[10][i] == 1)
        {
            if ((entradaSoma11 == true) && (entradaSoma12 == true))
            {
                blocoSoma1 = true;
                //printf("entrei1");
            }
        }
        i = i + 1;
    }

    //Bloco Soma 2 nao usado
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[11][i] == 1)
        {
            entradaSoma21 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[12][i] == 1)
        {
            entradaSoma22 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[13][i] == 1)
        {
            if ((entradaSoma21 == true) && (entradaSoma22 == true))
                blocoSoma2 = true;
        }
        i = i + 1;
    }

    //Bloco Temp1 nao usado
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[18][i] == 1)
        {
            blocoTemp1 = true;
        }
        i = i + 1;
    }

    //Bloco Temp2 nao usado
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[23][i] == 1)
        {
            blocoTemp2 = true;
        }
        i = i + 1;
    }

    //Bloco Decisor 1 nao usado
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[26][i] == 1)
        {
            blocoDecisor1 = true;
            entradaMux11 = true;
            entradaMux12 = true;
            entradaDec1 = true;
        }
        i = i + 1;
    }

    //Bloco Decisor 2 nao usado
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[31][i] == 1)
        {
            blocoDecisor2 = true;
            entradaMux21 = true;
            entradaMux22 = true;
            entradaDec2 = true;
        }
        i = i + 1;
    }

    //bloco else
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[42][i] == 1)
        {
            entradaElseA = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[43][i] == 1)
        {
            entradaElseB = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[44][i] == 1)
        {
            entradaElseC = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[45][i] == 1)
        {
            entradaElseD = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        //printf("%d: %d\n", i, ligacoes[46][i]);
        if (ligacoes[46][i] == 1)
        {

            if ((entradaElseA == true) && (entradaElseB == true) && (entradaElseC == true) && (entradaElseD == true))
            {
                //printf("true");
                blocoElse = true;
            }
        }
        i = i + 1;
    }

    //Bloco and
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[47][i] == 1)
        {
            entradaAnd1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[48][i] == 1)
        {
            entradaAnd2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[49][i] == 1)
        {
            if ((entradaAnd1 == true) && (entradaAnd2 == true))
                blocoAnd = true;
        }
        i = i + 1;
    }
    //Bloco or
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[50][i] == 1)
        {
            entradaOr1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[51][i] == 1)
        {
            entradaOr2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[52][i] == 1)
        {
            if ((entradaOr1 == true) && (entradaOr2 == true))
                blocoOr = true;
        }
        i = i + 1;
    }
    //Bloco maior
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[53][i] == 1)
        {
            //printf("ola");
            entradaMaior1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[54][i] == 1)
        {
            //printf("ola");
            entradaMaior2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[55][i] == 1)
        {
            if ((entradaMaior1 == true) && (entradaMaior2 == true))
            {
                blocoMaior = true;
            }
        }
        i = i + 1;
    }
    //Bloco menor
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[56][i] == 1)
        {

            entradaMenor1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[57][i] == 1)
        {

            entradaMenor2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[58][i] == 1)
        {
            if ((entradaMenor1 == true) && (entradaMenor2 == true))
            {
                blocoMenor = true;
            }
        }
        i = i + 1;
    }
    //Bloco igual
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[59][i] == 1)
        {

            entradaIgual1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[60][i] == 1)
        {

            entradaIgual2 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[61][i] == 1)
        {
            if ((entradaIgual1 == true) && (entradaIgual2 == true))
            {
                blocoIgual = true;
            }
        }
        i = i + 1;
    }

    //Bloco trans
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[62][i] == 1)
        {

            entradaTrans1 = true;
        }
        i = i + 1;
    }
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[63][i] == 1)
        {
            if ((entradaTrans1 == true))
            {
                blocoTrans = true;
            }
        }
        i = i + 1;
    }
    //----------------------------------------------------------------------
    //
    //
    //
    //Blocos temporizadores
    //
    //
    //
    //----------------------------------------------------------------------

    //Bloco Temp1
    //Temporizador1 - Entrada
    i = 1;
    while (i < rc + 1)
    {
        if ((ligacoes[18][i] == 1) && (matrixValores[18][i] >= 50))
        {
            if (antigoTemp1 == false)
            {
                activateTemp1 = true;
                //TimerTemp1.Interval = StrToInt(K1.Text);
                //TimerTemp1.Enabled = true;
                blocoTemp1 = false;
                antigoTemp1 = true;
                //ShowMessage('Temp 1 Ativa-se');
            }
        }
        if ((ligacoes[18][i] == 1) && (matrixValores[18][i] < 50))
        {
            antigoTemp1 = false;
            blocoTemp1 = false;
        }
        i = i + 1;
    }

    //Temporizador1 - Saida
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[19][i] == 1)
        {
            if (activateTemp1 == true)
            {
                matrixValores[19][i] = 100;
                matrixValores[i][19] = 100;
            }
            if (activateTemp1 == false)
            {
                matrixValores[19][i] = 0;
                matrixValores[i][19] = 0;
            }
        }
        i = i + 1;
    }
    blocoTemp1 = false;

    //Bloco Temp2
    //Temporizador2 - Entrada
    i = 1;
    while (i < rc + 1)
    {
        if ((ligacoes[22][i] == 1) && (matrixValores[22][i] >= 50))
        {
            if (antigoTemp2 == false)
            {
                activateTemp2 = true;
                //TimerTemp2.Interval = StrToInt(K2.Text);
                //TimerTemp2.Enabled = true;
                blocoTemp2 = false;
                antigoTemp2 = true;
                //ShowMessage('Temp 1 Ativa-se');
            }
        }
        if ((ligacoes[22][i] == 1) && (matrixValores[22][i] < 50))
        {
            antigoTemp2 = false;
            blocoTemp2 = false;
        }
        i = i + 1;
    }

    //Temporizador1 - Saida
    i = 1;
    while (i < rc + 1)
    {
        if (ligacoes[23][i] == 1)
        {
            if (activateTemp2 == true)
            {
                matrixValores[23][i] = 100;
                matrixValores[i][23] = 100;
            }
            if (activateTemp2 == false)
            {
                matrixValores[23][i] = 0;
                matrixValores[i][23] = 0;
            }
        }
        i = i + 1;
    }
    blocoTemp2 = false;

    while ((blocoInv1 == true) || (blocoInv2 == true) || (blocoSoma1 == true) || (blocoSoma2 == true) || (blocoTemp1 == true) || (blocoDecisor1 == true) || (blocoDecisor2 == true) || (blocoTemp2 == true) || (blocoProduto == true) || (blocoIf == true) || (blocoElse == true) || (blocoAnd == true) || (blocoOr == true) || (blocoMaior == true) || (blocoMenor == true) || (blocoIgual == true) || (blocoTrans== true))
    {

        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco inversor
        //
        //
        //
        //----------------------------------------------------------------------
        //Bloco Inversor 1
        if (blocoInv1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][6] == 1) || (ligacoes[2][6] == 1) ||
                    (ligacoes[3][6] == 1) || (ligacoes[4][6] == 1) ||
                    (ligacoes[5][6] == 1) || (ligacoes[14][6] == 1) ||
                    (ligacoes[25][6] == 1) || (ligacoes[30][6] == 1) ||
                    (ligacoes[35][6] == 1) || (ligacoes[36][6] == 1) ||
                    (ligacoes[37][6] == 1) || (ligacoes[63][6] == 1) ||
                    (ligacoes[64][6] == 1) || (ligacoes[65][6] == 1) ||
                    (ligacoes[66][6] == 1) || (ligacoes[67][6] == 1) ||
                    (ligacoes[68][6] == 1) || (ligacoes[69][6] == 1) ||
                    (ligacoes[70][6] == 1) || (ligacoes[71][6] == 1) ||
                    (ligacoes[72][6] == 1) || (ligacoes[73][6] == 1) ||
                    (ligacoes[74][6] == 1) || (ligacoes[75][6] == 1) ||
                    (ligacoes[76][6] == 1) || (ligacoes[77][6] == 1) ||
                    (ligacoes[78][6] == 1) || (ligacoes[79][6] == 1) ||
                    (ligacoes[79][6] == 1) || (ligacoes[80][6] == 1) ||
                    (ligacoes[81][6] == 1) || (ligacoes[82][6] == 1) ||
                    (ligacoes[83][6] == 1))
            {
                i = 1;
                while (i < rc + 1)
                {
                    if (ligacoes[6][i] == 1)
                    {
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[7][j] == 1)
                            {
                                matrixValores[7][j] = -matrixValores[6][i];
                                matrixValores[j][7] = -matrixValores[6][i];
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoInv1 = false;
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[6][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[10][i];
                            matrixValores[j][7] = -matrixValores[10][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[6][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[13][i];
                            matrixValores[j][7] = -matrixValores[13][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida do inversor2
            if (ligacoes[6][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[16][i];
                            matrixValores[j][7] = -matrixValores[16][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida do temporizador1
            if (ligacoes[6][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[19][i];
                            matrixValores[j][7] = -matrixValores[19][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida do temporizador2
            if (ligacoes[6][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[23][i];
                            matrixValores[j][7] = -matrixValores[23][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida do decisor1
            if (ligacoes[6][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[29][i];
                            matrixValores[j][7] = -matrixValores[29][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }

            //Caso em que vem da saida do decisor2
            if (ligacoes[6][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[34][i];
                            matrixValores[j][7] = -matrixValores[34][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //Caso em que vem da saida do if
            if (ligacoes[6][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[41][i];
                            matrixValores[j][7] = -matrixValores[41][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[6][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[46][i];
                            matrixValores[j][7] = -matrixValores[46][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            if (ligacoes[6][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[49][i];
                            matrixValores[j][7] = -matrixValores[49][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[6][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[51][i];
                            matrixValores[j][7] = -matrixValores[51][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[6][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[51][i];
                            matrixValores[j][7] = -matrixValores[51][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[6][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[58][i];
                            matrixValores[j][7] = -matrixValores[58][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            if (ligacoes[6][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[7][i];
                            matrixValores[j][7] = -matrixValores[7][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[6][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[7][j] == 1)
                        {
                            matrixValores[7][j] = -matrixValores[63][i];
                            matrixValores[j][7] = -matrixValores[63][i];
                        }
                        j = j + 1;
                    }
                    blocoInv1 = false;
                }
            }
        }

        //Bloco Inversor 2
        if (blocoInv2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][15] == 1) || (ligacoes[2][15] == 1) ||
                    (ligacoes[3][15] == 1) || (ligacoes[4][15] == 1) ||
                    (ligacoes[5][15] == 1) || (ligacoes[14][15] == 1) ||
                    (ligacoes[25][15] == 1) || (ligacoes[30][15] == 1) ||
                    (ligacoes[35][15] == 1) || (ligacoes[36][15] == 1) ||
                    (ligacoes[37][15] == 1) || (ligacoes[63][15] == 1) ||
                    (ligacoes[64][15] == 1) || (ligacoes[65][15] == 1) ||
                    (ligacoes[66][15] == 1) || (ligacoes[67][15] == 1) ||
                    (ligacoes[68][15] == 1) || (ligacoes[69][15] == 1) ||
                    (ligacoes[70][15] == 1) || (ligacoes[71][15] == 1) ||
                    (ligacoes[72][15] == 1) || (ligacoes[73][15] == 1) ||
                    (ligacoes[74][15] == 1) || (ligacoes[75][15] == 1) ||
                    (ligacoes[76][15] == 1) || (ligacoes[77][15] == 1) ||
                    (ligacoes[78][15] == 1) || (ligacoes[79][15] == 1) ||
                    (ligacoes[79][15] == 1) || (ligacoes[80][15] == 1) ||
                    (ligacoes[81][15] == 1) || (ligacoes[82][15] == 1) ||
                    (ligacoes[83][15]==1))
            {
                i = 1;
                while (i < rc + 1)
                {
                    if (ligacoes[15][i] == 1)
                    {
                        j = 1;
                        while (j < rc + 1)

                        {
                            if (ligacoes[16][j] == 1)
                            {
                                matrixValores[16][j] = -matrixValores[15][i];
                                matrixValores[j][16] = -matrixValores[15][i];
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoInv2 = false;
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[15][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[10][i];
                            matrixValores[j][16] = -matrixValores[10][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[15][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[13][i];
                            matrixValores[j][16] = -matrixValores[13][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida do inversor1
            if (ligacoes[15][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[7][i];
                            matrixValores[j][16] = -matrixValores[7][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida do temporizador1
            if (ligacoes[15][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[19][i];
                            matrixValores[j][16] = -matrixValores[19][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida do temporizador2
            if (ligacoes[15][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[23][i];
                            matrixValores[j][16] = -matrixValores[23][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida do decisor1
            if (ligacoes[15][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[29][i];
                            matrixValores[j][16] = -matrixValores[29][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            //Caso em que vem da saida do decisor2
            if (ligacoes[15][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    j = 1;
                    while (j < rc + 1)

                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[34][i];
                            matrixValores[j][16] = -matrixValores[34][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }

            if (ligacoes[15][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[41][i];
                            matrixValores[j][16] = -matrixValores[41][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[15][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[46][i];
                            matrixValores[j][16] = -matrixValores[46][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[15][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[49][i];
                            matrixValores[j][16] = -matrixValores[49][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[15][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[51][i];
                            matrixValores[j][16] = -matrixValores[51][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[15][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[55][i];
                            matrixValores[j][16] = -matrixValores[55][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[15][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[58][i];
                            matrixValores[j][16] = -matrixValores[58][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[15][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[61][i];
                            matrixValores[j][16] = -matrixValores[61][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[15][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {

                    j = 1;
                    while (j < rc + 1)
                    {
                        if (ligacoes[16][j] == 1)
                        {
                            matrixValores[16][j] = -matrixValores[63][i];
                            matrixValores[j][16] = -matrixValores[63][i];
                        }
                        j = j + 1;
                    }
                    blocoInv2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos multiplicacao
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //Bloco prod11
        if (entradaProd1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][20] == 1) || (ligacoes[2][20] == 1) ||
                    (ligacoes[3][20] == 1) || (ligacoes[4][20] == 1) ||
                    (ligacoes[5][20] == 1) || (ligacoes[14][20] == 1) ||
                    (ligacoes[25][20] == 1) || (ligacoes[30][20] == 1) ||
                    (ligacoes[35][20] == 1) || (ligacoes[36][20] == 1) ||
                    (ligacoes[37][20] == 1) || (ligacoes[63][20] == 1) ||
                    (ligacoes[64][20] == 1) || (ligacoes[65][20] == 1) ||
                    (ligacoes[66][20] == 1) || (ligacoes[67][20] == 1) ||
                    (ligacoes[68][20] == 1) || (ligacoes[69][20] == 1) ||
                    (ligacoes[70][20] == 1) || (ligacoes[71][20] == 1) ||
                    (ligacoes[72][20] == 1) || (ligacoes[73][20] == 1) ||
                    (ligacoes[74][20] == 1) || (ligacoes[75][20] == 1) ||
                    (ligacoes[76][20] == 1) || (ligacoes[77][20] == 1) ||
                    (ligacoes[78][20] == 1) || (ligacoes[79][20] == 1) ||
                    (ligacoes[79][20] == 1) || (ligacoes[80][20] == 1) ||
                    (ligacoes[81][20] == 1) || (ligacoes[82][20] == 1) ||
                    (ligacoes[83][20] == 1))
            {
                //printf("1");
                entradaProd1 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[20][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaProd1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[20][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaProd1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[20][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[20][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[20][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[20][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[20][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[20][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaProd1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[20][0] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaProd1 = false;
                }
            }
            //Caso em que vem da saida do if
            if (ligacoes[20][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaProd1 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[20][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[20][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[20][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[20][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[20][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[20][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaProd1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[20][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaProd1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos multiplicacao
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //Bloco prod2
        if (entradaProd2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][38] == 1) || (ligacoes[2][38] == 1) ||
                    (ligacoes[3][38] == 1) || (ligacoes[4][38] == 1) ||
                    (ligacoes[5][38] == 1) || (ligacoes[14][38] == 1) ||
                    (ligacoes[25][38] == 1) || (ligacoes[30][38] == 1) ||
                    (ligacoes[35][38] == 1) || (ligacoes[36][38] == 1) ||
                    (ligacoes[37][38] == 1) || (ligacoes[63][38] == 1) ||
                    (ligacoes[64][38] == 1) || (ligacoes[65][38] == 1) ||
                    (ligacoes[66][38] == 1) || (ligacoes[67][38] == 1) ||
                    (ligacoes[68][38] == 1) || (ligacoes[69][38] == 1) ||
                    (ligacoes[70][38] == 1) || (ligacoes[71][38] == 1) ||
                    (ligacoes[72][38] == 1) || (ligacoes[73][38] == 1) ||
                    (ligacoes[74][38] == 1) || (ligacoes[75][38] == 1) ||
                    (ligacoes[76][38] == 1) || (ligacoes[77][38] == 1) ||
                    (ligacoes[78][38] == 1) || (ligacoes[79][38] == 1) ||
                    (ligacoes[79][38] == 1) || (ligacoes[80][38] == 1) ||
                    (ligacoes[81][38] == 1) || (ligacoes[82][38] == 1) ||
                    (ligacoes[83][38] == 1))
            {
                //printf("2");
                entradaProd2 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[38][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[38][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[38][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[38][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[38][18] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[38][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[38][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[38][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaProd2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[38][0] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaProd2 = false;
                }
            }
            //Caso em que vem da saida do if
            if (ligacoes[38][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaProd2 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[38][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[38][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[38][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[38][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[28][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[28][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaProd2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[28][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaProd2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos multiplicacao
        //realiza multiplicacao
        //
        //
        //----------------------------------------------------------------------
        if (blocoProduto == true)
        {
            if ((entradaProd1 == false) && (entradaProd2 == false))
            {
                i = 1;
                while (i < rc + 1)
                {
                    if (ligacoes[i][20] == 1)
                    {
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][38] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][21] == 1)
                                    {
                                        //printf("3");
                                        matrixValores[aux][21] = matrixValores[i][20] * matrixValores[j][38];
                                        matrixValores[21][aux] = matrixValores[i][20] * matrixValores[j][38];
                                        if (matrixValores[aux][21] > 100)
                                        {
                                            matrixValores[aux][21] = 100;
                                            matrixValores[21][aux] = 100;
                                        }
                                        if (matrixValores[aux][21] < -100)
                                        {
                                            matrixValores[aux][21] = -100;
                                            matrixValores[21][aux] = -100;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoProduto = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos soma
        //verifica se entradas ja estao prontas
        //
        //
        //----------------------------------------------------------------------
        //Bloco Soma11
        if (entradaSoma11 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][8] == 1) || (ligacoes[2][8] == 1) ||
                    (ligacoes[3][8] == 1) || (ligacoes[4][8] == 1) ||
                    (ligacoes[5][8] == 1) || (ligacoes[14][8] == 1) ||
                    (ligacoes[25][8] == 1) || (ligacoes[30][8] == 1) ||
                    (ligacoes[35][8] == 1) || (ligacoes[36][8] == 1) ||
                    (ligacoes[37][8] == 1) || (ligacoes[63][8] == 1) ||
                    (ligacoes[64][8] == 1) || (ligacoes[65][8] == 1) ||
                    (ligacoes[66][8] == 1) || (ligacoes[67][8] == 1) ||
                    (ligacoes[68][8] == 1) || (ligacoes[69][8] == 1) ||
                    (ligacoes[70][8] == 1) || (ligacoes[71][8] == 1) ||
                    (ligacoes[72][8] == 1) || (ligacoes[73][8] == 1) ||
                    (ligacoes[74][8] == 1) || (ligacoes[75][8] == 1) ||
                    (ligacoes[76][8] == 1) || (ligacoes[77][8] == 1) ||
                    (ligacoes[78][8] == 1) || (ligacoes[79][8] == 1) ||
                    (ligacoes[79][8] == 1) || (ligacoes[80][8] == 1) ||
                    (ligacoes[81][8] == 1) || (ligacoes[82][8] == 1) ||
                    (ligacoes[83][8] == 1))
            {
                entradaSoma11 = false;
                //ROS_INFO("ENTRADA1");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[8][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaSoma11 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[8][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaSoma11 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[8][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[8][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[8][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[8][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[8][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[8][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaSoma11 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[8][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaSoma11 = false;
                }
            }
            //Caso em que vem da saida do if
            if (ligacoes[8][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaSoma11 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[8][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[8][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[8][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[8][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[8][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[8][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaSoma11 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[8][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaSoma11 = false;
                }
            }
        }

        //Bloco Soma12
        if (entradaSoma12 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][9] == 1) || (ligacoes[2][9] == 1) ||
                    (ligacoes[3][9] == 1) || (ligacoes[4][9] == 1) ||
                    (ligacoes[5][9] == 1) || (ligacoes[14][9] == 1) ||
                    (ligacoes[25][9] == 1) || (ligacoes[30][9] == 1) ||
                    (ligacoes[35][9] == 1) || (ligacoes[36][9] == 1) ||
                    (ligacoes[37][9] == 1) || (ligacoes[63][9] == 1) ||
                    (ligacoes[64][9] == 1) || (ligacoes[65][9] == 1) ||
                    (ligacoes[66][9] == 1) || (ligacoes[67][9] == 1) ||
                    (ligacoes[68][9] == 1) || (ligacoes[69][9] == 1) ||
                    (ligacoes[70][9] == 1) || (ligacoes[71][9] == 1) ||
                    (ligacoes[72][9] == 1) || (ligacoes[73][9] == 1) ||
                    (ligacoes[74][9] == 1) || (ligacoes[75][9] == 1) ||
                    (ligacoes[76][9] == 1) || (ligacoes[77][9] == 1) ||
                    (ligacoes[78][9] == 1) || (ligacoes[79][9] == 1) ||
                    (ligacoes[79][9] == 1) || (ligacoes[80][9] == 1) ||
                    (ligacoes[81][9] == 1) || (ligacoes[82][9] == 1) ||
                    (ligacoes[83][9] == 1))
            {
                entradaSoma12 = false;
                //ROS_INFO("ENTRADA2");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[9][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[9][0] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[9][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[9][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[9][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[9][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[9][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[9][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaSoma12 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[9][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaSoma12 = false;
                }
            }
            if (ligacoes[9][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaSoma12 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[9][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[9][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[9][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[9][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[9][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[9][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaSoma12 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[9][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaSoma12 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos soma
        //realiza soma
        //
        //
        //----------------------------------------------------------------------
        //Bloco Soma 1
        if (blocoSoma1 == true)
        {
            if ((entradaSoma11 == false) && (entradaSoma12 == false))
            {
                i = 1;
                while (i < rc + 1)
                {
                    if (ligacoes[i][8] == 1)
                    {
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][9] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][10] == 1)
                                    {
                                        //ROS_INFO("realiza soma");
                                        matrixValores[aux][10] = matrixValores[i][8] + matrixValores[j][9];
                                        matrixValores[10][aux] = matrixValores[i][8] + matrixValores[j][9];
                                        if (matrixValores[aux][10] > 100)
                                        {
                                            matrixValores[aux][10] = 100;
                                            matrixValores[10][aux] = 100;
                                        }
                                        if (matrixValores[aux][10] < -100)
                                        {
                                            matrixValores[aux][10] = -100;
                                            matrixValores[10][aux] = -100;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoSoma1 = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos soma
        //verifica se entradas ja estao prontas
        //
        //
        //----------------------------------------------------------------------
        //Bloco Soma21
        if (entradaSoma21 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][11] == 1) || (ligacoes[2][11] == 1) ||
                    (ligacoes[3][11] == 1) || (ligacoes[4][11] == 1) ||
                    (ligacoes[5][11] == 1) || (ligacoes[14][11] == 1) ||
                    (ligacoes[25][11] == 1) || (ligacoes[30][11] == 1) ||
                    (ligacoes[35][11] == 1) || (ligacoes[36][11] == 1) ||
                    (ligacoes[37][11] == 1) || (ligacoes[63][11] == 1) ||
                    (ligacoes[64][11] == 1) || (ligacoes[65][11] == 1) ||
                    (ligacoes[66][11] == 1) || (ligacoes[67][11] == 1) ||
                    (ligacoes[68][11] == 1) || (ligacoes[69][11] == 1) ||
                    (ligacoes[70][11] == 1) || (ligacoes[71][11] == 1) ||
                    (ligacoes[72][11] == 1) || (ligacoes[73][11] == 1) ||
                    (ligacoes[74][11] == 1) || (ligacoes[75][11] == 1) ||
                    (ligacoes[76][11] == 1) || (ligacoes[77][11] == 1) ||
                    (ligacoes[78][11] == 1) || (ligacoes[79][11] == 1) ||
                    (ligacoes[79][11] == 1) || (ligacoes[80][11] == 1) ||
                    (ligacoes[81][11] == 1) || (ligacoes[82][11] == 1) ||
                    (ligacoes[83][11] == 1))
            {
                entradaSoma21 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[11][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[11][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[11][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[11][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[11][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[11][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[11][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[11][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaSoma21 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[11][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaSoma21 = false;
                }
            }
            if (ligacoes[11][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaSoma21 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[11][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[11][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[11][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[11][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[11][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[11][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaSoma21 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[11][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaSoma21 = false;
                }
            }
        }

        //Bloco Soma22
        if (entradaSoma22 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][12] == 1) || (ligacoes[2][12] == 1) ||
                    (ligacoes[3][12] == 1) || (ligacoes[4][12] == 1) ||
                    (ligacoes[5][12] == 1) || (ligacoes[14][12] == 1) ||
                    (ligacoes[25][12] == 1) || (ligacoes[30][12] == 1) ||
                    (ligacoes[35][12] == 1) || (ligacoes[36][12] == 1) ||
                    (ligacoes[37][12] == 1) || (ligacoes[63][12] == 1) ||
                    (ligacoes[64][12] == 1) || (ligacoes[65][12] == 1) ||
                    (ligacoes[66][12] == 1) || (ligacoes[67][12] == 1) ||
                    (ligacoes[68][12] == 1) || (ligacoes[69][12] == 1) ||
                    (ligacoes[70][12] == 1) || (ligacoes[71][12] == 1) ||
                    (ligacoes[72][12] == 1) || (ligacoes[73][12] == 1) ||
                    (ligacoes[74][12] == 1) || (ligacoes[75][12] == 1) ||
                    (ligacoes[76][12] == 1) || (ligacoes[77][12] == 1) ||
                    (ligacoes[78][12] == 1) || (ligacoes[79][12] == 1) ||
                    (ligacoes[79][12] == 1) || (ligacoes[80][12] == 1) ||
                    (ligacoes[81][12] == 1) || (ligacoes[82][12] == 1) ||
                    (ligacoes[83][12] == 1))
            {
                entradaSoma22 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[12][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[12][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[12][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[12][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[12][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[12][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[12][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[12][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaSoma22 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[12][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaSoma22 = false;
                }
            }
            if (ligacoes[12][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaSoma22 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[12][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[12][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[12][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[12][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[12][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[12][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaSoma22 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[12][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaSoma22 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos soma
        //realiza soma
        //
        //
        //----------------------------------------------------------------------
        //Bloco Soma 2
        if (blocoSoma2 == true)
        {

            if ((entradaSoma21 == false) && (entradaSoma22 == false))
            {
                i = 1;
                while (i < rc + 1)
                {
                    if (ligacoes[i][11] == 1)
                    {
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][12] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][13] == 1)
                                    {
                                        matrixValores[aux][13] = matrixValores[i][11] + matrixValores[j][12];
                                        matrixValores[13][aux] = matrixValores[i][11] + matrixValores[j][12];
                                        if (matrixValores[aux][13] > 100)
                                        {
                                            matrixValores[aux][13] = 100;
                                            matrixValores[13][aux] = 100;
                                        }

                                        if (matrixValores[aux][13] < -100)
                                        {
                                            matrixValores[aux][13] = -100;
                                            matrixValores[13][aux] = -100;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoSoma2 = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos mux
        //verifica se entradas estao prontas
        //
        //
        //----------------------------------------------------------------------
        //Bloco Mux11

        if (entradaMux11 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][26] == 1) || (ligacoes[2][26] == 1) || (ligacoes[3][26] == 1) || (ligacoes[4][26] == 1) || (ligacoes[5][26] == 1) || (ligacoes[14][26] == 1) || (ligacoes[25][26] == 1) || (ligacoes[30][26] == 1) || (ligacoes[35][26] == 1) || (ligacoes[36][26] == 1) || (ligacoes[37][26] == 1))
            {
                entradaMux11 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[26][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[26][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[26][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[26][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[26][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[26][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[26][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[26][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMux11 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[26][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMux11 = false;
                }
            }
            if (ligacoes[26][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMux11 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[26][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[26][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[26][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[26][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[26][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[26][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMux11 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[26][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMux11 = false;
                }
            }
        }

        //Bloco Mux12
        if (entradaMux12 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][27] == 1) || (ligacoes[2][27] == 1) ||
                    (ligacoes[3][27] == 1) || (ligacoes[4][27] == 1) ||
                    (ligacoes[5][27] == 1) || (ligacoes[14][27] == 1) ||
                    (ligacoes[25][27] == 1) || (ligacoes[30][27] == 1) ||
                    (ligacoes[35][27] == 1) || (ligacoes[36][27] == 1) ||
                    (ligacoes[37][27] == 1) || (ligacoes[63][27] == 1) ||
                    (ligacoes[64][27] == 1) || (ligacoes[65][27] == 1) ||
                    (ligacoes[66][27] == 1) || (ligacoes[67][27] == 1) ||
                    (ligacoes[68][27] == 1) || (ligacoes[69][27] == 1) ||
                    (ligacoes[70][27] == 1) || (ligacoes[71][27] == 1) ||
                    (ligacoes[72][27] == 1) || (ligacoes[73][27] == 1) ||
                    (ligacoes[74][27] == 1) || (ligacoes[75][27] == 1) ||
                    (ligacoes[76][27] == 1) || (ligacoes[77][27] == 1) ||
                    (ligacoes[78][27] == 1) || (ligacoes[79][27] == 1) ||
                    (ligacoes[79][27] == 1) || (ligacoes[80][27] == 1) ||
                    (ligacoes[81][27] == 1) || (ligacoes[82][27] == 1) ||
                    (ligacoes[83][27] == 1))
            {
                entradaMux12 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[27][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[27][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[27][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[27][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[27][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[27][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[27][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[27][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMux12 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[27][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMux12 = false;
                }
            }
            if (ligacoes[27][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMux12 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[27][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[27][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[27][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[27][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[27][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[27][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMux12 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[27][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMux12 = false;
                }
            }
        }

        //Bloco Decisor 1
        if (entradaDec1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][28] == 1) || (ligacoes[2][28] == 1) ||
                    (ligacoes[3][28] == 1) || (ligacoes[4][28] == 1) ||
                    (ligacoes[5][28] == 1) || (ligacoes[14][28] == 1) ||
                    (ligacoes[25][28] == 1) || (ligacoes[30][28] == 1) ||
                    (ligacoes[35][28] == 1) || (ligacoes[36][28] == 1) ||
                    (ligacoes[37][28] == 1) || (ligacoes[63][28] == 1) ||
                    (ligacoes[64][28] == 1) || (ligacoes[65][28] == 1) ||
                    (ligacoes[66][28] == 1) || (ligacoes[67][28] == 1) ||
                    (ligacoes[68][28] == 1) || (ligacoes[69][28] == 1) ||
                    (ligacoes[70][28] == 1) || (ligacoes[71][28] == 1) ||
                    (ligacoes[72][28] == 1) || (ligacoes[73][28] == 1) ||
                    (ligacoes[74][28] == 1) || (ligacoes[75][28] == 1) ||
                    (ligacoes[76][28] == 1) || (ligacoes[77][28] == 1) ||
                    (ligacoes[78][28] == 1) || (ligacoes[79][28] == 1) ||
                    (ligacoes[79][28] == 1) || (ligacoes[80][28] == 1) ||
                    (ligacoes[81][28] == 1) || (ligacoes[82][28] == 1) ||
                    (ligacoes[83][28] == 1))
            {
                entradaDec1 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[28][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[28][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[28][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[28][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[28][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[28][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[28][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[28][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaDec1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[28][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaDec1 = false;
                }
            }

            if (ligacoes[28][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaDec1 = false;
                }
            }
            //Caso em que vem da saida do else
            if (ligacoes[28][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[28][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[28][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[28][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[28][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[28][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaDec1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[28][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaDec1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos mux
        //realiza decisao
        //
        //
        //----------------------------------------------------------------------
        //Bloco Multiplexador 1
        if (blocoDecisor1 == true)
        {

            if ((entradaMux11 == false) && (entradaMux12 == false) && (entradaDec1 == false))
            {
                i = 1;
                while (i < rc + 1)

                {
                    //Vejo o decisor
                    if (ligacoes[i][28] == 1)
                    {
                        if (matrixValores[i][28] >= 50)
                        {
                            j = 1;
                            while (j < rc + 1)

                            {
                                if (ligacoes[j][27] == 1)
                                {
                                    aux = 1;
                                    while (aux < rc + 1)

                                    {
                                        if (ligacoes[aux][29] == 1)
                                        {
                                            matrixValores[aux][29] = matrixValores[j][27];
                                            matrixValores[29][aux] = matrixValores[j][27];
                                        }
                                        aux = aux + 1;
                                    }
                                }
                                j = j + 1;
                            }
                        }
                        else
                        {
                            j = 1;
                            while (j < rc + 1)

                            {
                                if (ligacoes[j][26] == 1)
                                {
                                    aux = 1;
                                    while (aux < rc + 1)

                                    {
                                        if (ligacoes[aux][29] == 1)
                                        {
                                            matrixValores[aux][29] = matrixValores[j][26];
                                            matrixValores[29][aux] = matrixValores[j][26];
                                        }
                                        aux = aux + 1;
                                    }
                                }
                                j = j + 1;
                            }
                        }
                    }
                    i = i + 1;
                }
                blocoDecisor1 = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos mux 2
        //verifica se entradas estao prontas
        //
        //
        //----------------------------------------------------------------------
        //Bloco Mux21
        if (entradaMux21 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][31] == 1) || (ligacoes[2][31] == 1) ||
                    (ligacoes[3][31] == 1) || (ligacoes[4][31] == 1) ||
                    (ligacoes[5][31] == 1) || (ligacoes[14][31] == 1) ||
                    (ligacoes[25][31] == 1) || (ligacoes[30][31] == 1) ||
                    (ligacoes[35][31] == 1) || (ligacoes[36][31] == 1) ||
                    (ligacoes[37][31] == 1) || (ligacoes[63][31] == 1) ||
                    (ligacoes[64][31] == 1) || (ligacoes[65][31] == 1) ||
                    (ligacoes[66][31] == 1) || (ligacoes[67][31] == 1) ||
                    (ligacoes[68][31] == 1) || (ligacoes[69][31] == 1) ||
                    (ligacoes[70][31] == 1) || (ligacoes[71][31] == 1) ||
                    (ligacoes[72][31] == 1) || (ligacoes[73][31] == 1) ||
                    (ligacoes[74][31] == 1) || (ligacoes[75][31] == 1) ||
                    (ligacoes[76][31] == 1) || (ligacoes[77][31] == 1) ||
                    (ligacoes[78][31] == 1) || (ligacoes[79][31] == 1) ||
                    (ligacoes[79][31] == 1) || (ligacoes[80][31] == 1) ||
                    (ligacoes[81][31] == 1) || (ligacoes[82][31] == 1) ||
                    (ligacoes[83][31] == 1))
            {
                entradaMux21 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[31][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[31][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[31][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[31][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[31][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[31][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[31][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[31][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMux21 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[31][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[31][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[31][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[31][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[31][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[31][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[31][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[31][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMux21 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[31][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMux21 = false;
                }
            }
        }

        //Bloco Mux22
        if (entradaMux22 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][32] == 1) || (ligacoes[2][32] == 1) ||
                    (ligacoes[3][32] == 1) || (ligacoes[4][32] == 1) ||
                    (ligacoes[5][32] == 1) || (ligacoes[14][32] == 1) ||
                    (ligacoes[25][32] == 1) || (ligacoes[30][32] == 1) ||
                    (ligacoes[35][32] == 1) || (ligacoes[36][32] == 1) ||
                    (ligacoes[37][32] == 1) || (ligacoes[63][32] == 1) ||
                    (ligacoes[64][32] == 1) || (ligacoes[65][32] == 1) ||
                    (ligacoes[66][32] == 1) || (ligacoes[67][32] == 1) ||
                    (ligacoes[68][32] == 1) || (ligacoes[69][32] == 1) ||
                    (ligacoes[70][32] == 1) || (ligacoes[71][32] == 1) ||
                    (ligacoes[72][32] == 1) || (ligacoes[73][32] == 1) ||
                    (ligacoes[74][32] == 1) || (ligacoes[75][32] == 1) ||
                    (ligacoes[76][32] == 1) || (ligacoes[77][32] == 1) ||
                    (ligacoes[78][32] == 1) || (ligacoes[79][32] == 1) ||
                    (ligacoes[79][32] == 1) || (ligacoes[80][32] == 1) ||
                    (ligacoes[81][32] == 1) || (ligacoes[82][32] == 1) ||
                    (ligacoes[83][32] == 1))
            {
                entradaMux22 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[32][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[32][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[32][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[32][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[32][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[32][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[32][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[32][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMux22 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[32][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[32][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[32][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[32][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[32][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[32][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[32][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[32][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMux22 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[32][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMux22 = false;
                }
            }
        }

        //Bloco Decisor 2
        if (entradaDec2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][33] == 1) || (ligacoes[2][33] == 1) ||
                    (ligacoes[3][33] == 1) || (ligacoes[4][33] == 1) ||
                    (ligacoes[5][33] == 1) || (ligacoes[14][33] == 1) ||
                    (ligacoes[25][33] == 1) || (ligacoes[30][33] == 1) ||
                    (ligacoes[35][33] == 1) || (ligacoes[36][33] == 1) ||
                    (ligacoes[37][33] == 1) || (ligacoes[63][33] == 1) ||
                    (ligacoes[64][33] == 1) || (ligacoes[65][33] == 1) ||
                    (ligacoes[66][33] == 1) || (ligacoes[67][33] == 1) ||
                    (ligacoes[68][33] == 1) || (ligacoes[69][33] == 1) ||
                    (ligacoes[70][33] == 1) || (ligacoes[71][33] == 1) ||
                    (ligacoes[72][33] == 1) || (ligacoes[73][33] == 1) ||
                    (ligacoes[74][33] == 1) || (ligacoes[75][33] == 1) ||
                    (ligacoes[76][33] == 1) || (ligacoes[77][33] == 1) ||
                    (ligacoes[78][33] == 1) || (ligacoes[79][33] == 1) ||
                    (ligacoes[79][33] == 1) || (ligacoes[80][33] == 1) ||
                    (ligacoes[81][33] == 1) || (ligacoes[82][33] == 1) ||
                    (ligacoes[83][33] == 1))
            {
                entradaDec2 = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[33][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[33][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[33][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[33][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[33][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[33][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[33][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[33][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaDec2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[33][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[33][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[33][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[33][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[33][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[33][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[39][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[33][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaDec2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[33][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaDec2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos mux 2
        //realiza decisao
        //
        //
        //----------------------------------------------------------------------
        //Bloco Multiplexador 2
        if (blocoDecisor2 == true)
        {

            if ((entradaMux21 == false) && (entradaMux22 == false) && (entradaDec2 == false))
            {
                i = 1;
                while (i < rc + 1)
                {
                    //Vejo o decisor
                    if (ligacoes[i][33] == 1)
                    {
                        if (matrixValores[i][33] >= 50)
                        {
                            j = 1;
                            while (j < rc + 1)
                            {
                                {
                                    if (ligacoes[j][32] == 1)
                                    {
                                        aux = 1;
                                        while (aux < rc + 1)
                                        {
                                            if (ligacoes[aux][34] == 1)
                                            {
                                                matrixValores[aux][34] = matrixValores[j][32];
                                                matrixValores[34][aux] = matrixValores[j][32];
                                            }
                                            aux++;
                                        }
                                    }
                                    j++;
                                }
                            }
                        }
                        else
                        {
                            j = 1;
                            while (j < rc + 1)
                            {
                                if (ligacoes[j][31] == 1)
                                {
                                    aux = 1;
                                    while (aux < rc + 1)

                                    {
                                        if (ligacoes[aux][34] == 1)
                                        {
                                            matrixValores[aux][34] = matrixValores[j][31];
                                            matrixValores[34][aux] = matrixValores[j][31];
                                        }
                                        aux++;
                                    }
                                }
                                j++;
                            }
                        }
                    }
                    i++;
                }
                blocoDecisor2 = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco if
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //Bloco if11
        if (entradaIf1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][39] == 1) || (ligacoes[2][39] == 1) ||
                    (ligacoes[3][39] == 1) || (ligacoes[4][39] == 1) ||
                    (ligacoes[5][39] == 1) || (ligacoes[14][39] == 1) ||
                    (ligacoes[25][39] == 1) || (ligacoes[30][39] == 1) ||
                    (ligacoes[35][39] == 1) || (ligacoes[36][39] == 1) ||
                    (ligacoes[37][39] == 1) || (ligacoes[63][39] == 1) ||
                    (ligacoes[64][39] == 1) || (ligacoes[65][39] == 1) ||
                    (ligacoes[66][39] == 1) || (ligacoes[67][39] == 1) ||
                    (ligacoes[68][39] == 1) || (ligacoes[69][39] == 1) ||
                    (ligacoes[70][39] == 1) || (ligacoes[71][39] == 1) ||
                    (ligacoes[72][39] == 1) || (ligacoes[73][39] == 1) ||
                    (ligacoes[74][39] == 1) || (ligacoes[75][39] == 1) ||
                    (ligacoes[76][39] == 1) || (ligacoes[77][39] == 1) ||
                    (ligacoes[78][39] == 1) || (ligacoes[79][39] == 1) ||
                    (ligacoes[79][39] == 1) || (ligacoes[80][39] == 1) ||
                    (ligacoes[81][39] == 1) || (ligacoes[82][39] == 1) ||
                    (ligacoes[83][39] == 1))
            {
                entradaIf1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[39][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaIf1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[39][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaIf1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[39][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[39][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[39][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[39][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[39][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[39][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaIf1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[39][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[39][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[39][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[39][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[39][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[39][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[39][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaIf1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[39][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaIf1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos if
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //Bloco Soma12
        if (entradaIf2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][40] == 1) || (ligacoes[2][40] == 1) ||
                    (ligacoes[3][40] == 1) || (ligacoes[4][40] == 1) ||
                    (ligacoes[5][40] == 1) || (ligacoes[14][40] == 1) ||
                    (ligacoes[25][40] == 1) || (ligacoes[30][40] == 1) ||
                    (ligacoes[35][40] == 1) || (ligacoes[36][40] == 1) ||
                    (ligacoes[37][40] == 1)|| (ligacoes[63][40] == 1) ||
                    (ligacoes[64][40] == 1) || (ligacoes[65][40] == 1) ||
                    (ligacoes[66][40] == 1) || (ligacoes[67][40] == 1) ||
                    (ligacoes[68][40] == 1) || (ligacoes[69][40] == 1) ||
                    (ligacoes[70][40] == 1) || (ligacoes[71][40] == 1) ||
                    (ligacoes[72][40] == 1) || (ligacoes[73][40] == 1) ||
                    (ligacoes[74][40] == 1) || (ligacoes[75][40] == 1) ||
                    (ligacoes[76][40] == 1) || (ligacoes[77][40] == 1) ||
                    (ligacoes[78][40] == 1) || (ligacoes[79][40] == 1) ||
                    (ligacoes[79][40] == 1) || (ligacoes[80][40] == 1) ||
                    (ligacoes[81][40] == 1) || (ligacoes[82][40] == 1) ||
                    (ligacoes[83][40] == 1))
            {
                entradaIf2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[40][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[40][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[40][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[40][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[40][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[40][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[40][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[40][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaIf2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[40][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[40][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[40][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[40][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[40][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[40][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[40][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaIf2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[40][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaIf2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos if
        //realiza if
        //
        //
        //----------------------------------------------------------------------
        //Bloco if
        if (blocoIf == true)
        {

            if ((entradaIf1 == false) && (entradaIf2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][39] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][40] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][41] == 1)
                                    {
                                        matrixValores[aux][41] = matrixValores[i][39] * matrixValores[j][40];
                                        matrixValores[41][aux] = matrixValores[i][39] * matrixValores[j][40];
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoIf = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos else
        //verifica se entrada a ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada a else
        if (entradaElseA == true)
        {
            //printf("3\n");

            //Caso facil em que e um input imediato
            if ((ligacoes[1][42] == 1) || (ligacoes[2][42] == 1) ||
                    (ligacoes[3][42] == 1) || (ligacoes[4][42] == 1) ||
                    (ligacoes[5][42] == 1) || (ligacoes[14][42] == 1) ||
                    (ligacoes[25][42] == 1) || (ligacoes[30][42] == 1) ||
                    (ligacoes[35][42] == 1) || (ligacoes[36][42] == 1) ||
                    (ligacoes[37][42] == 1) || (ligacoes[63][42] == 1) ||
                    (ligacoes[64][42] == 1) || (ligacoes[65][42] == 1) ||
                    (ligacoes[66][42] == 1) || (ligacoes[67][42] == 1) ||
                    (ligacoes[68][42] == 1) || (ligacoes[69][42] == 1) ||
                    (ligacoes[70][42] == 1) || (ligacoes[71][42] == 1) ||
                    (ligacoes[72][42] == 1) || (ligacoes[73][42] == 1) ||
                    (ligacoes[74][42] == 1) || (ligacoes[75][42] == 1) ||
                    (ligacoes[76][42] == 1) || (ligacoes[77][42] == 1) ||
                    (ligacoes[78][42] == 1) || (ligacoes[79][42] == 1) ||
                    (ligacoes[79][42] == 1) || (ligacoes[80][42] == 1) ||
                    (ligacoes[81][42] == 1) || (ligacoes[82][42] == 1) ||
                    (ligacoes[83][42] == 1))
            {
                entradaElseA = false;
                //printf("1\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[42][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[42][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[42][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[42][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[42][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[42][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[42][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[42][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaElseA = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[42][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[42][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[42][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[42][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[42][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[42][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[42][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaElseA = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[42][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaElseA = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos else
        //verifica se entrada B ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada B
        if (entradaElseB == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][43] == 1) || (ligacoes[2][43] == 1) ||
                    (ligacoes[3][43] == 1) || (ligacoes[4][43] == 1) ||
                    (ligacoes[5][43] == 1) || (ligacoes[14][43] == 1) ||
                    (ligacoes[25][43] == 1) || (ligacoes[30][43] == 1) ||
                    (ligacoes[35][43] == 1) || (ligacoes[36][43] == 1) ||
                    (ligacoes[37][43] == 1) || (ligacoes[63][43] == 1) ||
                    (ligacoes[64][43] == 1) || (ligacoes[65][43] == 1) ||
                    (ligacoes[66][43] == 1) || (ligacoes[67][43] == 1) ||
                    (ligacoes[68][43] == 1) || (ligacoes[69][43] == 1) ||
                    (ligacoes[70][43] == 1) || (ligacoes[71][43] == 1) ||
                    (ligacoes[72][43] == 1) || (ligacoes[73][43] == 1) ||
                    (ligacoes[74][43] == 1) || (ligacoes[75][43] == 1) ||
                    (ligacoes[76][43] == 1) || (ligacoes[77][43] == 1) ||
                    (ligacoes[78][43] == 1) || (ligacoes[79][43] == 1) ||
                    (ligacoes[79][43] == 1) || (ligacoes[80][43] == 1) ||
                    (ligacoes[81][43] == 1) || (ligacoes[82][43] == 1) ||
                    (ligacoes[83][43] == 1))
            {
                entradaElseB = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[43][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[43][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[43][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[43][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[43][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[43][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[43][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[43][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaElseB = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[43][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[43][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[43][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[43][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[43][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[43][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[43][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaElseB = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[43][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaElseB = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos else
        //verifica se entrada C ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada C
        if (entradaElseC == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][44] == 1) || (ligacoes[2][44] == 1) ||
                    (ligacoes[3][44] == 1) || (ligacoes[4][44] == 1) ||
                    (ligacoes[5][44] == 1) || (ligacoes[14][44] == 1) ||
                    (ligacoes[25][44] == 1) || (ligacoes[30][44] == 1) ||
                    (ligacoes[35][44] == 1) || (ligacoes[36][44] == 1) ||
                    (ligacoes[37][44] == 1) || (ligacoes[63][44] == 1) ||
                    (ligacoes[64][44] == 1) || (ligacoes[65][44] == 1) ||
                    (ligacoes[66][44] == 1) || (ligacoes[67][44] == 1) ||
                    (ligacoes[68][44] == 1) || (ligacoes[69][44] == 1) ||
                    (ligacoes[70][44] == 1) || (ligacoes[71][44] == 1) ||
                    (ligacoes[72][44] == 1) || (ligacoes[73][44] == 1) ||
                    (ligacoes[74][44] == 1) || (ligacoes[75][44] == 1) ||
                    (ligacoes[76][44] == 1) || (ligacoes[77][44] == 1) ||
                    (ligacoes[78][44] == 1) || (ligacoes[79][44] == 1) ||
                    (ligacoes[79][44] == 1) || (ligacoes[80][44] == 1) ||
                    (ligacoes[81][44] == 1) || (ligacoes[82][44] == 1) ||
                    (ligacoes[83][44] == 1))
            {
                entradaElseC = false;
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[44][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[44][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[44][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[44][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[44][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[44][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[44][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[44][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaElseC = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[44][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[44][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[44][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[44][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[44][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[44][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[44][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaElseC = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[44][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaElseC = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos else
        //verifica se entrada D ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada D
        if (entradaElseD == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][45] == 1) || (ligacoes[2][45] == 1) ||
                    (ligacoes[3][45] == 1) || (ligacoes[4][45] == 1) ||
                    (ligacoes[5][45] == 1) || (ligacoes[14][45] == 1) ||
                    (ligacoes[25][45] == 1) || (ligacoes[30][45] == 1) ||
                    (ligacoes[35][45] == 1) || (ligacoes[36][45] == 1) ||
                    (ligacoes[37][45] == 1) || (ligacoes[63][45] == 1) ||
                    (ligacoes[64][45] == 1) || (ligacoes[65][45] == 1) ||
                    (ligacoes[66][45] == 1) || (ligacoes[67][45] == 1) ||
                    (ligacoes[68][45] == 1) || (ligacoes[69][45] == 1) ||
                    (ligacoes[70][45] == 1) || (ligacoes[71][45] == 1) ||
                    (ligacoes[72][45] == 1) || (ligacoes[73][45] == 1) ||
                    (ligacoes[74][45] == 1) || (ligacoes[75][45] == 1) ||
                    (ligacoes[76][45] == 1) || (ligacoes[77][45] == 1) ||
                    (ligacoes[78][45] == 1) || (ligacoes[79][45] == 1) ||
                    (ligacoes[79][45] == 1) || (ligacoes[80][45] == 1) ||
                    (ligacoes[81][45] == 1) || (ligacoes[82][45] == 1) ||
                    (ligacoes[83][45] == 1))
            {
                entradaElseD = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[45][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[45][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[45][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[45][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[45][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[45][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[45][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[45][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaElseD = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[45][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaElseD = false;
                }
            }

            //caso em que vem da saida do if
            if (ligacoes[45][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaElseD = false;
                }
            }

            //caso em que vem da saida do and
            if (ligacoes[45][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaElseD = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[45][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaElseD = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[45][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaElseD = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[45][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaElseD = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[45][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaElseD = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[45][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaElseD = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos else
        //realiza else
        //
        //
        //----------------------------------------------------------------------
        //Bloco else
        if (blocoElse == true)
        {

            if ((entradaElseA == false) && (entradaElseB == false) && (entradaElseC == false) && (entradaElseD == false)) //entradas todas prontas
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("%d : %d\n", i, ligacoes[i][42]);
                    if (ligacoes[i][42] == 1)
                    {

                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][43] == 1)
                            {
                                while (k < rc + 1)
                                {
                                    if (ligacoes[k][44] == 1)
                                    {
                                        while (l < rc + 1)
                                        {
                                            if (ligacoes[l][45] == 1)
                                            {
                                                aux = 1;
                                                while (aux < rc + 1)
                                                {

                                                    if (ligacoes[aux][46] == 1)
                                                    {
                                                        printf("%f %f %f\n", matrixValores[j][43], matrixValores[k][44], matrixValores[l][45]);

                                                        if (matrixValores[j][43] == 1 && matrixValores[k][44] == 0 && matrixValores[l][45] == 1)
                                                        {
                                                            printf("%f\n", matrixValores[i][42]);
                                                            matrixValores[aux][46] = matrixValores[i][42];
                                                            matrixValores[46][aux] = matrixValores[i][42];
                                                        }
                                                    }
                                                    aux = aux + 1;
                                                }
                                            }
                                            l++;
                                        }
                                    }
                                    k++;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                    blocoElse = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco and
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada and1
        if (entradaAnd1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][47] == 1) || (ligacoes[2][47] == 1) ||
                    (ligacoes[3][47] == 1) || (ligacoes[4][47] == 1) ||
                    (ligacoes[5][47] == 1) || (ligacoes[14][47] == 1) ||
                    (ligacoes[25][47] == 1) || (ligacoes[30][47] == 1) ||
                    (ligacoes[35][47] == 1) || (ligacoes[36][47] == 1) ||
                    (ligacoes[37][47] == 1) || (ligacoes[63][47] == 1) ||
                    (ligacoes[64][47] == 1) || (ligacoes[65][47] == 1) ||
                    (ligacoes[66][47] == 1) || (ligacoes[67][47] == 1) ||
                    (ligacoes[68][47] == 1) || (ligacoes[69][47] == 1) ||
                    (ligacoes[70][47] == 1) || (ligacoes[71][47] == 1) ||
                    (ligacoes[72][47] == 1) || (ligacoes[73][47] == 1) ||
                    (ligacoes[74][47] == 1) || (ligacoes[75][47] == 1) ||
                    (ligacoes[76][47] == 1) || (ligacoes[77][47] == 1) ||
                    (ligacoes[78][47] == 1) || (ligacoes[79][47] == 1) ||
                    (ligacoes[79][47] == 1) || (ligacoes[80][47] == 1) ||
                    (ligacoes[81][47] == 1) || (ligacoes[82][47] == 1) ||
                    (ligacoes[83][47] == 1))
            {
                entradaAnd1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[47][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaAnd1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[47][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaAnd1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[47][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[47][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[47][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[47][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[47][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[47][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaAnd1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[47][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[47][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[47][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[47][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[47][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[47][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[47][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaAnd1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[47][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaAnd1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos and
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada and2
        if (entradaAnd2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][48] == 1) || (ligacoes[2][48] == 1) ||
                    (ligacoes[3][48] == 1) || (ligacoes[4][48] == 1) ||
                    (ligacoes[5][48] == 1) || (ligacoes[14][48] == 1) ||
                    (ligacoes[25][48] == 1) || (ligacoes[30][48] == 1) ||
                    (ligacoes[35][48] == 1) || (ligacoes[36][48] == 1) ||
                    (ligacoes[37][48] == 1) || (ligacoes[63][48] == 1) ||
                    (ligacoes[64][48] == 1) || (ligacoes[65][48] == 1) ||
                    (ligacoes[66][48] == 1) || (ligacoes[67][48] == 1) ||
                    (ligacoes[68][48] == 1) || (ligacoes[69][48] == 1) ||
                    (ligacoes[70][48] == 1) || (ligacoes[71][48] == 1) ||
                    (ligacoes[72][48] == 1) || (ligacoes[73][48] == 1) ||
                    (ligacoes[74][48] == 1) || (ligacoes[75][48] == 1) ||
                    (ligacoes[76][48] == 1) || (ligacoes[77][48] == 1) ||
                    (ligacoes[78][48] == 1) || (ligacoes[79][48] == 1) ||
                    (ligacoes[79][48] == 1) || (ligacoes[80][48] == 1) ||
                    (ligacoes[81][48] == 1) || (ligacoes[82][48] == 1) ||
                    (ligacoes[83][48] == 1))
            {
                entradaAnd2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[48][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[48][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[48][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[48][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[48][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[48][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[48][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[48][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaAnd2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[48][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[48][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[48][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[48][51] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[48][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[48][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[48][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaAnd2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[48][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaAnd2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos and
        //realiza and
        //
        //
        //----------------------------------------------------------------------
        //Bloco and
        if (blocoAnd == true)
        {

            if ((entradaAnd1 == false) && (entradaAnd2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][47] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][48] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][49] == 1)
                                    {
                                        matrixValores[aux][49] = matrixValores[j][47] && matrixValores[i][48];
                                        matrixValores[49][aux] = matrixValores[j][47] && matrixValores[i][48];
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoAnd = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco or
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada or1
        if (entradaOr1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][50] == 1) || (ligacoes[2][50] == 1) ||
                    (ligacoes[3][50] == 1) || (ligacoes[4][50] == 1) ||
                    (ligacoes[5][50] == 1) || (ligacoes[14][50] == 1) ||
                    (ligacoes[25][50] == 1) || (ligacoes[30][50] == 1) ||
                    (ligacoes[35][50] == 1) || (ligacoes[36][50] == 1) ||
                    (ligacoes[37][50] == 1) || (ligacoes[63][50] == 1) ||
                    (ligacoes[64][50] == 1) || (ligacoes[65][50] == 1) ||
                    (ligacoes[66][50] == 1) || (ligacoes[67][50] == 1) ||
                    (ligacoes[68][50] == 1) || (ligacoes[69][50] == 1) ||
                    (ligacoes[70][50] == 1) || (ligacoes[71][50] == 1) ||
                    (ligacoes[72][50] == 1) || (ligacoes[73][50] == 1) ||
                    (ligacoes[74][50] == 1) || (ligacoes[75][50] == 1) ||
                    (ligacoes[76][50] == 1) || (ligacoes[77][50] == 1) ||
                    (ligacoes[78][50] == 1) || (ligacoes[79][50] == 1) ||
                    (ligacoes[79][50] == 1) || (ligacoes[80][50] == 1) ||
                    (ligacoes[81][50] == 1) || (ligacoes[82][50] == 1) ||
                    (ligacoes[83][50] == 1))
            {
                entradaOr1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[50][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaOr1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[50][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaOr1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[50][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[50][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[50][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[50][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[50][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[50][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaOr1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[50][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[50][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[50][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[50][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[50][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[50][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[50][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaOr1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[50][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaOr1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos or
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada or2
        if (entradaOr2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][51] == 1) || (ligacoes[2][51] == 1) ||
                    (ligacoes[3][51] == 1) || (ligacoes[4][51] == 1) ||
                    (ligacoes[5][51] == 1) || (ligacoes[14][51] == 1) ||
                    (ligacoes[25][51] == 1) || (ligacoes[30][51] == 1) ||
                    (ligacoes[35][51] == 1) || (ligacoes[36][51] == 1) ||
                    (ligacoes[37][51] == 1) || (ligacoes[63][51] == 1) ||
                    (ligacoes[64][51] == 1) || (ligacoes[65][51] == 1) ||
                    (ligacoes[66][51] == 1) || (ligacoes[67][51] == 1) ||
                    (ligacoes[68][51] == 1) || (ligacoes[69][51] == 1) ||
                    (ligacoes[70][51] == 1) || (ligacoes[71][51] == 1) ||
                    (ligacoes[72][51] == 1) || (ligacoes[73][51] == 1) ||
                    (ligacoes[74][51] == 1) || (ligacoes[75][51] == 1) ||
                    (ligacoes[76][51] == 1) || (ligacoes[77][51] == 1) ||
                    (ligacoes[78][51] == 1) || (ligacoes[79][51] == 1) ||
                    (ligacoes[79][51] == 1) || (ligacoes[80][51] == 1) ||
                    (ligacoes[81][51] == 1) || (ligacoes[82][51] == 1) ||
                    (ligacoes[83][51] == 1))
            {
                entradaOr2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[51][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[51][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[51][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[51][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[51][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[51][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[51][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[51][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaOr2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[51][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[51][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[51][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[51][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[51][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[51][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[51][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaOr2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[51][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaOr2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos or
        //realiza or
        //
        //
        //----------------------------------------------------------------------
        //Bloco or
        if (blocoOr == true)
        {

            if ((entradaOr1 == false) && (entradaOr2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][50] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][51] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][52] == 1)
                                    {

                                        matrixValores[aux][52] = matrixValores[j][51] || matrixValores[i][50];
                                        matrixValores[52][aux] = matrixValores[j][51] || matrixValores[i][50];
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoOr = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco or
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada or1
        if (entradaMaior1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][53] == 1) || (ligacoes[2][53] == 1) ||
                    (ligacoes[3][53] == 1) || (ligacoes[4][53] == 1) ||
                    (ligacoes[5][53] == 1) || (ligacoes[14][53] == 1) ||
                    (ligacoes[25][53] == 1) || (ligacoes[30][53] == 1) ||
                    (ligacoes[35][53] == 1) || (ligacoes[36][53] == 1) ||
                    (ligacoes[37][53] == 1) || (ligacoes[63][53] == 1) ||
                    (ligacoes[64][53] == 1) || (ligacoes[65][53] == 1) ||
                    (ligacoes[66][53] == 1) || (ligacoes[67][53] == 1) ||
                    (ligacoes[68][53] == 1) || (ligacoes[69][53] == 1) ||
                    (ligacoes[70][53] == 1) || (ligacoes[71][53] == 1) ||
                    (ligacoes[72][53] == 1) || (ligacoes[73][53] == 1) ||
                    (ligacoes[74][53] == 1) || (ligacoes[75][53] == 1) ||
                    (ligacoes[76][53] == 1) || (ligacoes[77][53] == 1) ||
                    (ligacoes[78][53] == 1) || (ligacoes[79][53] == 1) ||
                    (ligacoes[79][53] == 1) || (ligacoes[80][53] == 1) ||
                    (ligacoes[81][53] == 1) || (ligacoes[82][53] == 1) ||
                    (ligacoes[83][53] == 1))
            {
                entradaMaior1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[53][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMaior1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[53][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMaior1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[53][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[53][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[53][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[53][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[53][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[53][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMaior1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[53][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[53][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[53][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[53][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[53][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[53][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[53][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMaior1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[53][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMaior1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos or
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada or2
        if (entradaMaior2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][54] == 1) || (ligacoes[2][54] == 1) ||
                    (ligacoes[3][54] == 1) || (ligacoes[4][54] == 1) ||
                    (ligacoes[5][54] == 1) || (ligacoes[14][54] == 1) ||
                    (ligacoes[25][54] == 1) || (ligacoes[30][54] == 1) ||
                    (ligacoes[35][54] == 1) || (ligacoes[36][54] == 1) ||
                    (ligacoes[37][54] == 1) || (ligacoes[63][54] == 1) ||
                    (ligacoes[64][54] == 1) || (ligacoes[65][54] == 1) ||
                    (ligacoes[66][54] == 1) || (ligacoes[67][54] == 1) ||
                    (ligacoes[68][54] == 1) || (ligacoes[69][54] == 1) ||
                    (ligacoes[70][54] == 1) || (ligacoes[71][54] == 1) ||
                    (ligacoes[72][54] == 1) || (ligacoes[73][54] == 1) ||
                    (ligacoes[74][54] == 1) || (ligacoes[75][54] == 1) ||
                    (ligacoes[76][54] == 1) || (ligacoes[77][54] == 1) ||
                    (ligacoes[78][54] == 1) || (ligacoes[79][54] == 1) ||
                    (ligacoes[79][54] == 1) || (ligacoes[80][54] == 1) ||
                    (ligacoes[81][54] == 1) || (ligacoes[82][54] == 1) ||
                    (ligacoes[83][54] == 1))
            {
                entradaMaior2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[54][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[54][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[54][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[54][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[54][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[54][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[54][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[54][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMaior2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[54][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[54][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[54][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[54][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[54][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[54][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[54][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMaior2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[54][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMaior2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos maior
        //realiza maior
        //
        //
        //----------------------------------------------------------------------
        //Bloco maior
        if (blocoMaior == true)
        {

            if ((entradaMaior1 == false) && (entradaMaior2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][53] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][54] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][55] == 1)
                                    {
                                        if (matrixValores[i][53] > matrixValores[j][54])
                                        {
                                            matrixValores[aux][55] = 1;
                                            matrixValores[55][aux] = 1;
                                        }
                                        else if (matrixValores[i][53] <= matrixValores[j][54])
                                        {
                                            matrixValores[aux][55] = 0;
                                            matrixValores[55][aux] = 0;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoMaior = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco menor
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada menor1
        if (entradaMenor1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][56] == 1) || (ligacoes[2][56] == 1) ||
                    (ligacoes[3][56] == 1) || (ligacoes[4][56] == 1) ||
                    (ligacoes[5][56] == 1) || (ligacoes[14][56] == 1) ||
                    (ligacoes[25][56] == 1) || (ligacoes[30][56] == 1) ||
                    (ligacoes[35][56] == 1) || (ligacoes[36][56] == 1) ||
                    (ligacoes[37][56] == 1) || (ligacoes[63][56] == 1) ||
                    (ligacoes[64][56] == 1) || (ligacoes[65][56] == 1) ||
                    (ligacoes[66][56] == 1) || (ligacoes[67][56] == 1) ||
                    (ligacoes[68][56] == 1) || (ligacoes[69][56] == 1) ||
                    (ligacoes[70][56] == 1) || (ligacoes[71][56] == 1) ||
                    (ligacoes[72][56] == 1) || (ligacoes[73][56] == 1) ||
                    (ligacoes[74][56] == 1) || (ligacoes[75][56] == 1) ||
                    (ligacoes[76][56] == 1) || (ligacoes[77][56] == 1) ||
                    (ligacoes[78][56] == 1) || (ligacoes[79][56] == 1) ||
                    (ligacoes[79][56] == 1) || (ligacoes[80][56] == 1) ||
                    (ligacoes[81][56] == 1) || (ligacoes[82][56] == 1) ||
                    (ligacoes[83][56] == 1))
            {
                entradaMenor1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[56][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMenor1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[56][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMenor1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[56][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[56][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[56][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[56][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[56][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[56][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMenor1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[56][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[56][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[56][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[56][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[56][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[56][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[56][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMenor1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[56][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMenor1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos menor
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada menor2
        if (entradaMenor2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][57] == 1) || (ligacoes[2][57] == 1) ||
                    (ligacoes[3][57] == 1) || (ligacoes[4][57] == 1) ||
                    (ligacoes[5][57] == 1) || (ligacoes[14][57] == 1) ||
                    (ligacoes[25][57] == 1) || (ligacoes[30][57] == 1) ||
                    (ligacoes[35][57] == 1) || (ligacoes[36][57] == 1) ||
                    (ligacoes[37][57] == 1) || (ligacoes[63][57] == 1) ||
                    (ligacoes[64][57] == 1) || (ligacoes[65][57] == 1) ||
                    (ligacoes[66][57] == 1) || (ligacoes[67][57] == 1) ||
                    (ligacoes[68][57] == 1) || (ligacoes[69][57] == 1) ||
                    (ligacoes[70][57] == 1) || (ligacoes[71][57] == 1) ||
                    (ligacoes[72][57] == 1) || (ligacoes[73][57] == 1) ||
                    (ligacoes[74][57] == 1) || (ligacoes[75][57] == 1) ||
                    (ligacoes[76][57] == 1) || (ligacoes[77][57] == 1) ||
                    (ligacoes[78][57] == 1) || (ligacoes[79][57] == 1) ||
                    (ligacoes[79][57] == 1) || (ligacoes[80][57] == 1) ||
                    (ligacoes[81][57] == 1) || (ligacoes[82][57] == 1) ||
                    (ligacoes[83][57] == 1))
            {
                entradaMenor2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[57][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[57][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[57][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[57][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[57][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[57][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[57][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[57][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaMenor2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[57][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[57][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[57][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[57][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[57][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[57][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaMenor2 = false;
                }
            }

            //caso em que vem da saida do igual
            if (ligacoes[57][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaMenor2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[57][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaMenor2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos menor
        //realiza menor
        //
        //
        //----------------------------------------------------------------------
        //Bloco menor
        if (blocoMenor == true)
        {

            if ((entradaMenor1 == false) && (entradaMenor2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][56] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][57] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][58] == 1)
                                    {
                                        if (matrixValores[i][56] < matrixValores[j][57])
                                        {

                                            matrixValores[aux][58] = 1;
                                            matrixValores[58][aux] = 1;
                                        }
                                        else if (matrixValores[i][56] >= matrixValores[j][57])
                                        {

                                            matrixValores[aux][58] = 0;
                                            matrixValores[58][aux] = 0;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoMenor = false;
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco igual
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada igual1
        if (entradaIgual1 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][59] == 1) || (ligacoes[2][59] == 1) ||
                    (ligacoes[3][59] == 1) || (ligacoes[4][59] == 1) ||
                    (ligacoes[5][59] == 1) || (ligacoes[14][59] == 1) ||
                    (ligacoes[25][59] == 1) || (ligacoes[30][59] == 1) ||
                    (ligacoes[35][59] == 1) || (ligacoes[36][59] == 1) ||
                    (ligacoes[37][59] == 1) || (ligacoes[63][59] == 1) ||
                    (ligacoes[64][59] == 1) || (ligacoes[65][59] == 1) ||
                    (ligacoes[66][59] == 1) || (ligacoes[67][59] == 1) ||
                    (ligacoes[68][59] == 1) || (ligacoes[69][59] == 1) ||
                    (ligacoes[70][59] == 1) || (ligacoes[71][59] == 1) ||
                    (ligacoes[72][59] == 1) || (ligacoes[73][59] == 1) ||
                    (ligacoes[74][59] == 1) || (ligacoes[75][59] == 1) ||
                    (ligacoes[76][59] == 1) || (ligacoes[77][59] == 1) ||
                    (ligacoes[78][59] == 1) || (ligacoes[79][59] == 1) ||
                    (ligacoes[79][59] == 1) || (ligacoes[80][59] == 1) ||
                    (ligacoes[81][59] == 1) || (ligacoes[82][59] == 1) ||
                    (ligacoes[83][59] == 1))
            {
                entradaIgual1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[59][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaIgual1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[59][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaIgual1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[59][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[59][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[59][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[59][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[59][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[59][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaIgual1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[59][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[59][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[59][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[59][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[59][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[59][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[59][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaIgual1 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[59][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaIgual1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos igual
        //verifica se entrada2 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada igual2
        if (entradaIgual2 == true)
        {

            //Caso facil em que e um input imediato
            if ((ligacoes[1][60] == 1) || (ligacoes[2][60] == 1) ||
                    (ligacoes[3][60] == 1) || (ligacoes[4][60] == 1) ||
                    (ligacoes[5][60] == 1) || (ligacoes[14][60] == 1) ||
                    (ligacoes[25][60] == 1) || (ligacoes[30][60] == 1) ||
                    (ligacoes[35][60] == 1) || (ligacoes[36][60] == 1) ||
                    (ligacoes[37][60] == 1) || (ligacoes[63][60] == 1) ||
                    (ligacoes[64][60] == 1) || (ligacoes[65][60] == 1) ||
                    (ligacoes[66][60] == 1) || (ligacoes[67][60] == 1) ||
                    (ligacoes[68][60] == 1) || (ligacoes[69][60] == 1) ||
                    (ligacoes[70][60] == 1) || (ligacoes[71][60] == 1) ||
                    (ligacoes[72][60] == 1) || (ligacoes[73][60] == 1) ||
                    (ligacoes[74][60] == 1) || (ligacoes[75][60] == 1) ||
                    (ligacoes[76][60] == 1) || (ligacoes[77][60] == 1) ||
                    (ligacoes[78][60] == 1) || (ligacoes[79][60] == 1) ||
                    (ligacoes[79][60] == 1) || (ligacoes[80][60] == 1) ||
                    (ligacoes[81][60] == 1) || (ligacoes[82][60] == 1) ||
                    (ligacoes[83][60] == 1))
            {
                entradaIgual2 = false;
                //printf("3\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[60][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da soma1
            if (ligacoes[60][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da soma2
            if (ligacoes[60][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[60][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[60][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[60][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[60][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[60][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaIgual2 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[60][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[60][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[60][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[60][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[60][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[60][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[60][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaIgual2 = false;
                }
            }
            //caso em que vem da saida do trans
            if (ligacoes[60][63] == 1)
            {
                //Esta pronto
                if (blocoTrans == false)
                {
                    entradaIgual2 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos igual
        //realiza igual
        //
        //
        //----------------------------------------------------------------------
        //Bloco igual
        if (blocoIgual == true)
        {

            if ((entradaIgual1 == false) && (entradaIgual2 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][59] == 1)
                    {
                        //printf("5\n");
                        j = 1;
                        while (j < rc + 1)
                        {
                            if (ligacoes[j][60] == 1)
                            {
                                aux = 1;
                                while (aux < rc + 1)
                                {
                                    if (ligacoes[aux][61] == 1)
                                    {
                                        if (matrixValores[i][59] == matrixValores[j][60])
                                        {
                                            matrixValores[aux][61] = 1;
                                            matrixValores[61][aux] = 1;
                                        }
                                        else if (matrixValores[i][59] != matrixValores[j][60])
                                        {
                                            matrixValores[aux][61] = 0;
                                            matrixValores[61][aux] = 0;
                                        }
                                    }
                                    aux = aux + 1;
                                }
                            }
                            j = j + 1;
                        }
                    }
                    i = i + 1;
                }
                blocoIgual = false;
            }
        }

        //----------------------------------------------------------------------
        //
        //
        //
        //Bloco transposição
        //verifica se entrada1 ja esta pronta
        //
        //
        //----------------------------------------------------------------------
        //entrada trans1
        if (entradaTrans1 == true)
        {

            //Caigualso facil em que e um input imediato
            if ((ligacoes[1][62] == 1) || (ligacoes[2][62] == 1) ||
                    (ligacoes[3][62] == 1) || (ligacoes[4][62] == 1) ||
                    (ligacoes[5][62] == 1) || (ligacoes[14][62] == 1) ||
                    (ligacoes[25][62] == 1) || (ligacoes[30][62] == 1) ||
                    (ligacoes[35][62] == 1) || (ligacoes[36][62] == 1) ||
                    (ligacoes[37][62] == 1) || (ligacoes[63][62] == 1) ||
                    (ligacoes[64][62] == 1) || (ligacoes[65][62] == 1) ||
                    (ligacoes[66][62] == 1) || (ligacoes[67][62] == 1) ||
                    (ligacoes[68][62] == 1) || (ligacoes[69][62] == 1) ||
                    (ligacoes[70][62] == 1) || (ligacoes[71][62] == 1) ||
                    (ligacoes[72][62] == 1) || (ligacoes[73][62] == 1) ||
                    (ligacoes[74][62] == 1) || (ligacoes[75][62] == 1) ||
                    (ligacoes[76][62] == 1) || (ligacoes[77][62] == 1) ||
                    (ligacoes[78][62] == 1) || (ligacoes[79][62] == 1) ||
                    (ligacoes[79][62] == 1) || (ligacoes[80][62] == 1) ||
                    (ligacoes[81][62] == 1) || (ligacoes[82][62] == 1) ||
                    (ligacoes[83][62] == 1))
            {
                entradaTrans1 = false;
                //printf("2\n");
            }

            //Caso em que vem da saida da inv1
            if (ligacoes[62][7] == 1)
            {
                //Esta pronto
                if (blocoInv1 == false)
                {
                    entradaTrans1 = false;
                }
            }
            //Caso em que vem da saida da soma1
            if (ligacoes[62][10] == 1)
            {
                //Esta pronto
                if (blocoSoma1 == false)
                {
                    entradaTrans1 = false;
                }
            }
            //Caso em que vem da saida da soma2
            if (ligacoes[62][13] == 1)
            {
                //Esta pronto
                if (blocoSoma2 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida da inv2
            if (ligacoes[62][16] == 1)
            {
                //Esta pronto
                if (blocoInv2 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida da temp1
            if (ligacoes[62][19] == 1)
            {
                //Esta pronto
                if (blocoTemp1 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida da temp2
            if (ligacoes[62][23] == 1)
            {
                //Esta pronto
                if (blocoTemp2 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida da decisor1
            if (ligacoes[62][29] == 1)
            {
                //Esta pronto
                if (blocoDecisor1 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida da decisor2
            if (ligacoes[62][34] == 1)
            {
                //Esta pronto
                if (blocoDecisor2 == false)
                {
                    entradaTrans1 = false;
                }
            }

            //Caso em que vem da saida do produto
            if (ligacoes[62][21] == 1)
            {
                //Esta pronto
                if (blocoProduto == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do else
            if (ligacoes[62][46] == 1)
            {
                //Esta pronto
                if (blocoElse == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do if
            if (ligacoes[62][41] == 1)
            {
                //Esta pronto
                if (blocoIf == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do and
            if (ligacoes[62][49] == 1)
            {
                //Esta pronto
                if (blocoAnd == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do or
            if (ligacoes[62][52] == 1)
            {
                //Esta pronto
                if (blocoOr == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do maior
            if (ligacoes[62][55] == 1)
            {
                //Esta pronto
                if (blocoMaior == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do menor
            if (ligacoes[62][58] == 1)
            {
                //Esta pronto
                if (blocoMenor == false)
                {
                    entradaTrans1 = false;
                }
            }
            //caso em que vem da saida do igual
            if (ligacoes[62][61] == 1)
            {
                //Esta pronto
                if (blocoIgual == false)
                {
                    entradaTrans1 = false;
                }
            }
        }
        //----------------------------------------------------------------------
        //
        //
        //
        //Blocos trans
        //realiza trans
        //
        //
        //----------------------------------------------------------------------
        //Bloco trans
        if (blocoIgual == true)
        {

            if ((entradaTrans1 == false))
            {

                i = 1;
                while (i < rc + 1)
                {
                    //printf("4\n");
                    if (ligacoes[i][62] == 1)
                    {
                        aux = 1;
                        while (aux < rc + 1)
                        {
                            if (ligacoes[aux][63] == 1)
                            {
                                ligacoes[i][aux]=1;
                                ligacoes[aux][i]=1;
                                matrixValores[i][aux]=matrixValores[i][62];
                                matrixValores[aux][i]=matrixValores[i][62];
                            }
                            aux = aux + 1;
                        }

                    }
                    i = i + 1;
                }
                blocoIgual = false;
            }
        }
    }

    //atualiza valores dos motores
    //Motor Esquerdo
    i = 1;
    motorEsq = 0;
    while (i < rc + 1)
    {
        if (ligacoes[i][17] == 1)
        {
            //ROS_INFO("ATUALIZA VALOR MOTOR");
            motorEsq = matrixValores[i][17];
        }
        i = i + 1;
    }

    //Motor Direito
    i = 1;
    motorDir = 0;
    while (i < rc + 1)
    {
        if (ligacoes[i][24] == 1)
        {
            // printf("%d", i);
            motorDir = matrixValores[i][24];
        }
        i = i + 1;
    }
    velocidades[1]=motorDir;
    velocidades[2]=motorEsq;
}

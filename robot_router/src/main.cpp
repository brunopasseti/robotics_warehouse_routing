#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <vector>
#include "Util.h"
#include "readData.h"

using namespace std;

double** matrizAdj;	 // matriz de adjacencia
int dimension;		 // quantidade total de vertices
int tamanhoSolucao;
const int iteracoesMaxima = 50;
int counterSwaps[iteracoesMaxima] = {0},
	counterReinsertion[iteracoesMaxima] = {0},
	counterReinsertion_2[iteracoesMaxima] = {0},
	counterReinsertion_3[iteracoesMaxima] = {0},
	counterTwo_Opt[iteracoesMaxima] = {0},
	counterDoubleBridge[iteracoesMaxima] = {0};
double tempoTotalSwap = 0, tempoTotalReinsertion = 0,
	   tempoTotalReinsertion_2 = 0, tempoTotalReinsertion_3 = 0,
	   tempoTotalTwo_Opt = 0;

struct tLocais {
	int distancia;
	int i;
	int localInsercao;
};

int GenerateRandomNumber(int tamanho);
void printData();
void MelhorInsercao(vector<int>& solucao,
					int escolhido,
					vector<tLocais>& melhorDistancia,
					vector<int>& conjuntoLocais);
void ExcluirValorEscolhido(vector<int>& conjuntoDeLocais, int localInsercao);
void InsercaoMaisBarata(vector<int>& conjuntoDeLocais,
						vector<int>& solucao,
						vector<tLocais>& melhorCaminho);
void Limitar_Variacoes_Dos_Indices(int& indiceInicial, int& indiceFinal);
void EscreverResultadosNosArquivos(fstream& File,
								   int* counters,
								   int iteracoesMaxima,
								   char* Nomearquivo);
void VerificarLimitesParaIndices(int& indiceInicial1,
								 int& indiceFinal1,
								 int& indiceInicial2,
								 int& indiceFinal2);
bool Ordena(tLocais a, tLocais b);
double Algoritmo_RVND(vector<int>& solucao,
					  double distancia,
					  int interacaoNoMomento);
double Reinsertion(vector<int>& solucao, double distancia, int tamanho);
double Swap(vector<int>& solucao, double distancia);
double Two_OPT(vector<int>& solucao, double distancia);
double DoubleBridge_Pertubation(vector<int>& solucao, double distancia);
//+++++++++++++++++++++++++++++ ROBOTICA ++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++ MAIN ++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv) {
	readData(argc, argv, &dimension, &matrizAdj);
	// printData();
	tamanhoSolucao = dimension + 1;

	srand((unsigned)time(0));

	vector<int> solucaoFinal;
	vector<int> auxConjuntoDeNos;
	int custoFinal = INT_MAX;

	fstream fileSwap, fileReinsertion, fileReinsertion_2, fileReinsertion_3,
		fileTwo_Opt, fileDoubleBridge;

	for (int j = 2; j <= dimension; j++)
		auxConjuntoDeNos.push_back(j);

	double tempo_inicial_TSP = cpuTime();

	for (int i = 0; i < iteracoesMaxima; i++) {
		vector<int> solucao{1, 1}, conjuntoDeLocais = auxConjuntoDeNos;
		int tamanho, escolhido, distancia = 0,
								quantidadeDeInsercoesIniciais = 2;
		bool usados[solucao.size() + 1] = {};

		for (int j = 0; j < quantidadeDeInsercoesIniciais; j++) {
			tamanho = conjuntoDeLocais.size();
			escolhido = GenerateRandomNumber(tamanho);
			solucao.emplace(solucao.begin() + 1,
							conjuntoDeLocais[escolhido - 1]);
			conjuntoDeLocais.erase(conjuntoDeLocais.begin() + escolhido - 1);
		}

		int quantidadeDeNosIniciais = quantidadeDeInsercoesIniciais + 1;
		for (int j = 0; j < quantidadeDeNosIniciais; j++) {
			distancia += matrizAdj[solucao[j]][solucao[j + 1]];
		}

		while (!conjuntoDeLocais.empty()) {
			vector<tLocais> melhorCaminho;
			InsercaoMaisBarata(conjuntoDeLocais, solucao, melhorCaminho);

			tamanho = melhorCaminho.size();
			int alfa = (rand() % 6);
			int quantidade = ((alfa / 10.0) * tamanho) + 1;
			int escolhaAleatoriaDoVertice = rand() % quantidade;

			sort(melhorCaminho.begin(), melhorCaminho.end(), Ordena);

			solucao.emplace(
				solucao.begin() + melhorCaminho[escolhaAleatoriaDoVertice].i,
				melhorCaminho[escolhaAleatoriaDoVertice].localInsercao);
			distancia += melhorCaminho[escolhaAleatoriaDoVertice].distancia;

			ExcluirValorEscolhido(
				conjuntoDeLocais,
				melhorCaminho[escolhaAleatoriaDoVertice].localInsercao);
		}

		distancia = Algoritmo_RVND(solucao, distancia, i);
		int iterMaxIls;
		if (dimension >= 150) {
			iterMaxIls = dimension / 2;
		} else {
			iterMaxIls = dimension;
		}

		while (iterMaxIls--) {
			int novaDistancia = distancia;
			vector<int> copiaSolucao = solucao;

			novaDistancia =
				DoubleBridge_Pertubation(copiaSolucao, novaDistancia);
			novaDistancia = Algoritmo_RVND(copiaSolucao, novaDistancia, i);

			if (novaDistancia < distancia) {
				counterDoubleBridge[i]++;
				if (dimension >= 150) {
					iterMaxIls = dimension / 2;
				} else {
					iterMaxIls = dimension;
				}
				distancia = novaDistancia;
				solucao = copiaSolucao;
			}
		}

		if (distancia < custoFinal) {
			solucaoFinal = solucao;
			custoFinal = distancia;
		}
	}

	double tempo_final_TSP = cpuTime() - tempo_inicial_TSP;

	return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ END MAIN
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

double VerificarOMelhor(double distancia, double novaDistancia) {
	if (distancia < novaDistancia)
		return novaDistancia;
	return distancia;
}

void printData() {
	cout << "dimension: " << dimension << endl;
	for (size_t i = 1; i <= dimension; i++) {
		for (size_t j = 1; j <= dimension; j++) {
			cout << matrizAdj[i][j] << " ";
		}
		cout << endl;
	}
}

int GenerateRandomNumber(int tamanho) {
	return (rand() % tamanho) + 1;
}

bool Ordena(tLocais a, tLocais b) {
	return a.distancia < b.distancia;
}

void ExcluirValorEscolhido(vector<int>& conjuntoDeLocais, int localInsercao) {
	for (int i = 0; i < conjuntoDeLocais.size(); i++) {
		if (conjuntoDeLocais[i] == localInsercao) {
			conjuntoDeLocais.erase(conjuntoDeLocais.begin() + i);
			break;
		}
	}
}

double Swap(vector<int>& solucao, double distancia) {
	tLocais melhorSwap;
	melhorSwap.i = 0;
	melhorSwap.localInsercao = 0;
	melhorSwap.distancia = 0;

	for (int i = 1; i < dimension; i++) {
		double CustoDeRetirarArcoIniciail =
			-matrizAdj[solucao[i]][solucao[i - 1]] -
			matrizAdj[solucao[i]][solucao[i + 1]];

		for (int j = i + 2; j < dimension - 1; j++) {
			double CustoTotalDeSwap;

			CustoTotalDeSwap = CustoDeRetirarArcoIniciail;
			CustoTotalDeSwap += -matrizAdj[solucao[j]][solucao[j - 1]] -
								matrizAdj[solucao[j]][solucao[j + 1]];
			CustoTotalDeSwap += matrizAdj[solucao[i]][solucao[j - 1]] +
								matrizAdj[solucao[i]][solucao[j + 1]];
			CustoTotalDeSwap += matrizAdj[solucao[j]][solucao[i - 1]] +
								matrizAdj[solucao[j]][solucao[i + 1]];

			if (CustoTotalDeSwap < melhorSwap.distancia) {
				melhorSwap.distancia = CustoTotalDeSwap;
				melhorSwap.i = i;
				melhorSwap.localInsercao = j;
			}
		}
	}

	distancia = distancia + melhorSwap.distancia;
	int aux = solucao[melhorSwap.i];
	solucao[melhorSwap.i] = solucao[melhorSwap.localInsercao];
	solucao[melhorSwap.localInsercao] = aux;

	return distancia;
}

double Reinsertion(vector<int>& solucao, double distancia, int tamanho) {
	tLocais melhorReinsercao;
	melhorReinsercao.distancia = 0;
	int quantidadeDeIteracoes = tamanhoSolucao - tamanho;

	for (int i = 1; i < quantidadeDeIteracoes; i++) {
		int posicao = i + tamanho - 1;

		double CustoTotalDeTirarArcosIniciais =
			-matrizAdj[solucao[i - 1]][solucao[i]] -
			matrizAdj[solucao[posicao]][solucao[i + tamanho]] +
			matrizAdj[solucao[i + tamanho]][solucao[i - 1]];

		for (int j = i + tamanho; j < quantidadeDeIteracoes; j++) {
			double CustoTotalDeReinsercao =
				CustoTotalDeTirarArcosIniciais -
				matrizAdj[solucao[j]][solucao[j + 1]];
			CustoTotalDeReinsercao +=
				matrizAdj[solucao[j]][solucao[i]] +
				matrizAdj[solucao[j + 1]][solucao[posicao]];

			if (CustoTotalDeReinsercao < melhorReinsercao.distancia) {
				melhorReinsercao.distancia = CustoTotalDeReinsercao;
				melhorReinsercao.i = i;
				melhorReinsercao.localInsercao = j;
			}
		}

		for (int j = i - tamanho; j > 0; j--) {
			double CustoTotalDeReinsercao = CustoTotalDeTirarArcosIniciais;
			CustoTotalDeReinsercao += matrizAdj[solucao[j - 1]][solucao[i]] +
									  matrizAdj[solucao[posicao]][solucao[j]] -
									  matrizAdj[solucao[j]][solucao[j - 1]];

			if (CustoTotalDeReinsercao < melhorReinsercao.distancia) {
				melhorReinsercao.distancia = CustoTotalDeReinsercao;
				melhorReinsercao.i = i;
				melhorReinsercao.localInsercao = j - 1;
			}
		}
	}

	distancia = distancia + melhorReinsercao.distancia;

	if (melhorReinsercao.distancia < 0) {
		vector<int> aux;

		int j = 0;

		while (j < tamanhoSolucao) {
			if (melhorReinsercao.i == j)
				j = melhorReinsercao.i + tamanho;

			if (melhorReinsercao.localInsercao + 1 == j) {
				for (int x = 0; x < tamanho; x++) {
					aux.push_back(solucao[melhorReinsercao.i + x]);
				}
			}
			aux.push_back(solucao[j]);
			j++;
		}
		solucao = aux;
	}

	return distancia;
}

void InsercaoMaisBarata(vector<int>& conjuntoDeLocais,
						vector<int>& solucao,
						vector<tLocais>& melhorCaminho) {
	vector<int> auxLocais = conjuntoDeLocais;
	int tamanho, escolhido;

	while (!auxLocais.empty()) {
		tamanho = auxLocais.size();
		escolhido = GenerateRandomNumber(tamanho);
		MelhorInsercao(solucao, escolhido, melhorCaminho, auxLocais);
		auxLocais.erase(auxLocais.begin() + escolhido - 1);
	}
}

void MelhorInsercao(vector<int>& solucao,
					int escolhido,
					vector<tLocais>& melhorDistancia,
					vector<int>& conjuntoLocais) {
	tLocais local;
	int distancia;

	for (int i = 1; i < solucao.size(); i++) {
		distancia = matrizAdj[solucao[i - 1]][conjuntoLocais[escolhido - 1]] +
					matrizAdj[solucao[i]][conjuntoLocais[escolhido - 1]] -
					matrizAdj[solucao[i - 1]][solucao[i]];
		local.distancia = distancia;
		local.i = i;
		local.localInsercao = conjuntoLocais[escolhido - 1];

		melhorDistancia.push_back(local);
	}
}

double Two_OPT(vector<int>& solucao, double distancia) {
	tLocais melhorTwo_OPT;
	melhorTwo_OPT.distancia = 0;

	for (int i = 1; i < dimension; i++) {
		double CustoTotalDeTirarPrimeiroArco =
			matrizAdj[solucao[i]][solucao[i + 1]];

		for (int j = i + 2; j < dimension; j++) {
			double CustoTotalDois_OPT = -CustoTotalDeTirarPrimeiroArco -
										matrizAdj[solucao[j]][solucao[j + 1]];
			CustoTotalDois_OPT += matrizAdj[solucao[i]][solucao[j]] +
								  matrizAdj[solucao[i + 1]][solucao[j + 1]];

			if (CustoTotalDois_OPT < melhorTwo_OPT.distancia) {
				melhorTwo_OPT.distancia = CustoTotalDois_OPT;
				melhorTwo_OPT.i = i + 1;
				melhorTwo_OPT.localInsercao = j + 1;
			}
		}
	}

	distancia = distancia + melhorTwo_OPT.distancia;

	if (melhorTwo_OPT.distancia < 0) {
		vector<int> aux;
		for (int j = melhorTwo_OPT.localInsercao - 1; j >= melhorTwo_OPT.i;
			 j--) {
			aux.push_back(solucao[j]);
		}

		swap_ranges(solucao.begin() + melhorTwo_OPT.i,
					solucao.begin() + melhorTwo_OPT.localInsercao, aux.begin());
	}

	return distancia;
}

double Algoritmo_RVND(vector<int>& solucao,
					  double distancia,
					  int interacaoNoMomento) {
	vector<string> algoritmos{"swap", "reinsertion", "reinsertion-2",
							  "reinsertion-3", "two_opt"};
	vector<string> copiaAlgoritmos = algoritmos;

	while (1) {
		if (copiaAlgoritmos.empty())
			break;

		int escolhaDeAlgoritmoAleatoria = rand() % copiaAlgoritmos.size();
		double novaDistancia;

		if (algoritmos[escolhaDeAlgoritmoAleatoria] == "swap") {
			double startTimeSwap = cpuTime();

			novaDistancia = Swap(solucao, distancia);

			tempoTotalSwap += cpuTime() - startTimeSwap;
		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion") {
			int tamanhoDeBlocosParaReinsercao = 1;
			double startTimeReinsertion = cpuTime();

			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);

			tempoTotalReinsertion += cpuTime() - startTimeReinsertion;
		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-2") {
			int tamanhoDeBlocosParaReinsercao = 2;
			double startTimeReinsertion_2 = cpuTime();

			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);

			tempoTotalReinsertion_2 += cpuTime() - startTimeReinsertion_2;

		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-3") {
			int tamanhoDeBlocosParaReinsercao = 3;
			double startTimeReinsertion_3 = cpuTime();

			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);

			tempoTotalReinsertion_3 += cpuTime() - startTimeReinsertion_3;

		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "two_opt") {
			double startTimeTwo_Opt = cpuTime();

			novaDistancia = Two_OPT(solucao, distancia);

			tempoTotalTwo_Opt += cpuTime() - startTimeTwo_Opt;
		}

		if (distancia > novaDistancia) {
			if (algoritmos[escolhaDeAlgoritmoAleatoria] == "swap")
				counterSwaps[interacaoNoMomento]++;
			else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion")
				counterReinsertion[interacaoNoMomento]++;
			else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-2")
				counterReinsertion_2[interacaoNoMomento]++;
			else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-3")
				counterReinsertion_3[interacaoNoMomento]++;
			else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "two_opt")
				counterTwo_Opt[interacaoNoMomento]++;

			distancia = novaDistancia;
			copiaAlgoritmos = algoritmos;
		} else {
			copiaAlgoritmos.erase(copiaAlgoritmos.begin() +
								  escolhaDeAlgoritmoAleatoria);
		}
	}

	return distancia;
}

double DoubleBridge_Pertubation(vector<int>& solucao, double distancia) {
	vector<int> copiaDaSolucao;
	int indiceInicial1 = (rand() % (dimension - 1)) + 1;
	int indiceFinal1 = (rand() % (dimension - 1)) + 1;
	int indiceInicial2 = (rand() % (dimension - 1)) + 1;
	int indiceFinal2 = (rand() % (dimension - 1)) + 1;

	Limitar_Variacoes_Dos_Indices(indiceInicial1, indiceFinal1);
	Limitar_Variacoes_Dos_Indices(indiceInicial2, indiceFinal2);

	VerificarLimitesParaIndices(indiceInicial1, indiceFinal1, indiceInicial2,
								indiceFinal2);

	if (indiceFinal1 + 1 != indiceInicial2 &&
		indiceInicial1 != indiceFinal2 + 1) {
		distancia -=
			(matrizAdj[solucao[indiceFinal2 + 1]][solucao[indiceFinal2]] +
			 matrizAdj[solucao[indiceInicial2]][solucao[indiceInicial2 - 1]]);
		distancia -=
			(matrizAdj[solucao[indiceFinal1 + 1]][solucao[indiceFinal1]] +
			 matrizAdj[solucao[indiceInicial1]][solucao[indiceInicial1 - 1]]);
		distancia +=
			(matrizAdj[solucao[indiceFinal1 + 1]][solucao[indiceFinal2]] +
			 matrizAdj[solucao[indiceInicial1 - 1]][solucao[indiceInicial2]]);
		distancia +=
			(matrizAdj[solucao[indiceFinal2 + 1]][solucao[indiceFinal1]] +
			 matrizAdj[solucao[indiceInicial2 - 1]][solucao[indiceInicial1]]);
	} else {
		if (indiceFinal1 + 1 == indiceInicial2) {
			distancia -=
				matrizAdj[solucao[indiceFinal2 + 1]][solucao[indiceFinal2]];
			distancia -=
				(matrizAdj[solucao[indiceInicial2]][solucao[indiceFinal1]] +
				 matrizAdj[solucao[indiceInicial1]]
						  [solucao[indiceInicial1 - 1]]);
			distancia +=
				(matrizAdj[solucao[indiceInicial1]][solucao[indiceFinal2]] +
				 matrizAdj[solucao[indiceInicial1 - 1]]
						  [solucao[indiceInicial2]]);
			distancia +=
				matrizAdj[solucao[indiceFinal2 + 1]][solucao[indiceFinal1]];
		} else if (indiceFinal2 + 1 == indiceInicial1) {
			distancia -=
				matrizAdj[solucao[indiceInicial2]][solucao[indiceInicial2 - 1]];
			distancia -=
				(matrizAdj[solucao[indiceFinal1 + 1]][solucao[indiceFinal1]] +
				 matrizAdj[solucao[indiceInicial1]][solucao[indiceFinal2]]);
			distancia +=
				(matrizAdj[solucao[indiceFinal1]][solucao[indiceInicial2]] +
				 matrizAdj[solucao[indiceFinal2]][solucao[indiceFinal1 + 1]]);
			distancia +=
				matrizAdj[solucao[indiceInicial2 - 1]][solucao[indiceInicial1]];
		}
	}

	int j = 0;
	while (j < tamanhoSolucao) {
		if (indiceInicial2 == j) {
			for (int x = indiceInicial1; x <= indiceFinal1; x++) {
				copiaDaSolucao.push_back(solucao[x]);
			}
			j = indiceFinal2;
		} else if (indiceInicial1 == j) {
			for (int x = indiceInicial2; x <= indiceFinal2; x++) {
				copiaDaSolucao.push_back(solucao[x]);
			}
			j = indiceFinal1;
		} else
			copiaDaSolucao.push_back(solucao[j]);
		j++;
	}

	solucao = copiaDaSolucao;
	return distancia;
}

void Limitar_Variacoes_Dos_Indices(int& indiceInicial, int& indiceFinal) {
	if (indiceFinal < indiceInicial) {
		int aux = indiceFinal;
		indiceFinal = indiceInicial;
		indiceInicial = aux;
	}

	int limiteMaximo = min(15, dimension / 4);

	if ((indiceFinal - indiceInicial) > limiteMaximo)
		indiceFinal = indiceInicial + limiteMaximo;
}

void EscreverResultadosNosArquivos(fstream& File,
								   int* counters,
								   int iteracoesMaxima,
								   char* NomeArquivo) {
	File << "------------------------------------------ " << NomeArquivo
		 << " ------------------------------------------" << endl;
	for (int i = 0; i < iteracoesMaxima; i++) {
		File << "Quantidade de Vezes Que Houve Melhora a Cada Iteração " << i
			 << " = " << counters[i] << endl;
	}
	File << "__________________________________________ "
		 << "END"
		 << " __________________________________________" << endl;
}

void VerificarLimitesParaIndices(int& indiceInicial1,
								 int& indiceFinal1,
								 int& indiceInicial2,
								 int& indiceFinal2) {
	if (indiceInicial1 <= indiceInicial2 && indiceInicial2 <= indiceFinal1) {
		int quantidadeDeNosContidosEntreIndiceInicial2_IndiceFinal1 =
			(indiceFinal1 - indiceInicial2) + 1;

		if (indiceInicial2 +
				quantidadeDeNosContidosEntreIndiceInicial2_IndiceFinal1 >=
			dimension) {
			// int quantidadeDeNosContidosEntreIndiceInicial2_IndiceInicial1 =
			// (indiceInicial2 - indiceInicial1) + 1;
			int quantidadeDeVerticesEntreIndiceInicial1_IndiceFinal2 =
				(indiceFinal2 - indiceInicial1) + 1;
			indiceFinal2 -=
				quantidadeDeVerticesEntreIndiceInicial1_IndiceFinal2;
			int novoIndice2 =
				indiceInicial2 -
				quantidadeDeVerticesEntreIndiceInicial1_IndiceFinal2;

			if (novoIndice2 < 2) {
				indiceInicial2 = 1;
			} else {
				indiceInicial2 = novoIndice2;
			}
			// indiceInicial2 -=
			// quantidadeDeNosContidosEntreIndiceInicial2_IndiceInicial1;
		} else {
			indiceInicial2 +=
				quantidadeDeNosContidosEntreIndiceInicial2_IndiceFinal1;
			indiceFinal2 +=
				quantidadeDeNosContidosEntreIndiceInicial2_IndiceFinal1;

			if (indiceFinal2 >= dimension)
				indiceFinal2 = dimension - 1;
		}
	} else if (indiceInicial1 <= indiceFinal2 &&
			   indiceInicial1 > indiceInicial2) {
		int quantidadeDeVerticesEntreIndiceInicial1_IndiceFinal2 =
			indiceFinal2 - indiceInicial1 + 1;
		indiceFinal2 -= quantidadeDeVerticesEntreIndiceInicial1_IndiceFinal2;
	}
}

#include "TSPAction.h"

std::vector<int> TSPAction::ils_rvnd() {
	const int iteracoesMaxima = 50;
    std::vector<int> solucaoFinal;
	std::vector<int> auxConjuntoDeNos;
	int custoFinal = INT_MAX;

	for (int j = 1; j < dimension; j++)
		auxConjuntoDeNos.push_back(j);
	ROS_INFO("All nodes created!");

	for (int i = 0; i < iteracoesMaxima; i++) {

        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }

		std::vector<int> solucao{0, 0}, conjuntoDeLocais = auxConjuntoDeNos;
		int tamanho, escolhido, distancia = 0,
								quantidadeDeInsercoesIniciais = 2;
		std::vector<bool> usados = std::vector<bool>(solucao.size() + 1, false);

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

		//ROS_INFO("Construction started! Custo: %d", distancia);
		while (!conjuntoDeLocais.empty()) {
			std::vector<tLocais> melhorCaminho;
			InsercaoMaisBarata(conjuntoDeLocais, solucao, melhorCaminho);

			tamanho = melhorCaminho.size();
			int alfa = (rand() % 6);
			int quantidade = ((alfa / 10.0) * tamanho) + 1;
			int escolhaAleatoriaDoVertice = rand() % quantidade;

			std::sort(melhorCaminho.begin(), melhorCaminho.end(), Ordena);

			solucao.emplace(
				solucao.begin() + melhorCaminho[escolhaAleatoriaDoVertice].i,
				melhorCaminho[escolhaAleatoriaDoVertice].localInsercao);
			distancia += melhorCaminho[escolhaAleatoriaDoVertice].distancia;

			ExcluirValorEscolhido(
				conjuntoDeLocais,
				melhorCaminho[escolhaAleatoriaDoVertice].localInsercao);
		}
		//ROS_INFO("Construction ended!");

		distancia = Algoritmo_RVND(solucao, distancia, i);
		int iterMaxIls;
		if (dimension >= 150) {
			iterMaxIls = dimension / 2;
		} else {
			iterMaxIls = dimension;
		}
		ROS_INFO("Custo %d", distancia);
		while (iterMaxIls--) {
			int novaDistancia = distancia;
			std::vector<int> copiaSolucao = solucao;

			//ROS_INFO("Perturbation started!");
			novaDistancia = DoubleBridge_Pertubation(
				copiaSolucao, novaDistancia);
			//ROS_INFO("Perturbation ended!\nLocalSearch started!");
			novaDistancia = Algoritmo_RVND(copiaSolucao, novaDistancia, i);
			//ROS_INFO("LocalSearch ended!");
			if (novaDistancia < distancia) {
				//ROS_INFO("Custo anterior: %d, Novo Custo: %d", distancia, novaDistancia);
				if (dimension >= 150) {
					iterMaxIls = dimension / 2;
				} else {
					iterMaxIls = dimension;
				}
				distancia = novaDistancia;
				solucao = copiaSolucao;
			}
		}
		//ROS_INFO("LocalSearch ended!");
		if (distancia < custoFinal) {
			solucaoFinal = solucao;
			custoFinal = distancia;
            feedback_.sequence = solucaoFinal;
            as_.publishFeedback(feedback_);
		}
	}

	return solucaoFinal;
}

int TSPAction::GenerateRandomNumber(int tamanho) {
	return (rand() % tamanho) + 1;
}

bool TSPAction::Ordena(tLocais a, tLocais b) {
	return a.distancia < b.distancia;
}

void TSPAction::ExcluirValorEscolhido(std::vector<int>& conjuntoDeLocais, int localInsercao) {
	for (int i = 0; i < conjuntoDeLocais.size(); i++) {
		if (conjuntoDeLocais[i] == localInsercao) {
			conjuntoDeLocais.erase(conjuntoDeLocais.begin() + i);
			break;
		}
	}
}

double TSPAction::Swap(std::vector<int>& solucao,
			double distancia) {
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

double TSPAction::Reinsertion(std::vector<int>& solucao,
				   double distancia,
				   int tamanho) {
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
		std::vector<int> aux;

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

void TSPAction::InsercaoMaisBarata(std::vector<int>& conjuntoDeLocais,
						std::vector<int>& solucao,
						std::vector<tLocais>& melhorCaminho) {
	std::vector<int> auxLocais = conjuntoDeLocais;
	int tamanho, escolhido;

	while (!auxLocais.empty()) {
		tamanho = auxLocais.size();
		escolhido = GenerateRandomNumber(tamanho);
		MelhorInsercao(solucao, escolhido, melhorCaminho, auxLocais);
		auxLocais.erase(auxLocais.begin() + escolhido - 1);
	}
}

void TSPAction::MelhorInsercao(std::vector<int>& solucao,
					int escolhido,
					std::vector<tLocais>& melhorDistancia,
					std::vector<int>& conjuntoLocais) {
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

double TSPAction::Two_OPT(std::vector<int>& solucao,
			   double distancia) {
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
		std::vector<int> aux;
		for (int j = melhorTwo_OPT.localInsercao - 1; j >= melhorTwo_OPT.i;
			 j--) {
			aux.push_back(solucao[j]);
		}

		swap_ranges(solucao.begin() + melhorTwo_OPT.i,
					solucao.begin() + melhorTwo_OPT.localInsercao, aux.begin());
	}

	return distancia;
}

double TSPAction::Algoritmo_RVND(std::vector<int>& solucao,
					  double distancia,
					  int interacaoNoMomento) {
	std::vector<std::string> algoritmos{"swap", "reinsertion", "two_opt"};
	std::vector<std::string> copiaAlgoritmos = algoritmos;

	while (1) {
		if (copiaAlgoritmos.empty())
			break;

		int escolhaDeAlgoritmoAleatoria = rand() % copiaAlgoritmos.size();
		double novaDistancia;

		if (algoritmos[escolhaDeAlgoritmoAleatoria] == "swap") {
			novaDistancia = Swap(solucao, distancia);

		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion") {
			int tamanhoDeBlocosParaReinsercao = 1;
			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);

		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-2") {
			int tamanhoDeBlocosParaReinsercao = 2;

			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);


		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "reinsertion-3") {
			int tamanhoDeBlocosParaReinsercao = 3;

			novaDistancia =
				Reinsertion(solucao, distancia, tamanhoDeBlocosParaReinsercao);


		} else if (algoritmos[escolhaDeAlgoritmoAleatoria] == "two_opt") {

			novaDistancia = Two_OPT(solucao, distancia);

		}

		if (distancia > novaDistancia) {
			//if(novaDistancia < 0)
			//	ROS_INFO("Novo custo %d, Vizinhança: %d", novaDistancia, (int) escolhaDeAlgoritmoAleatoria);
		
			distancia = novaDistancia;
			copiaAlgoritmos = algoritmos;
		} else {
			copiaAlgoritmos.erase(copiaAlgoritmos.begin() +
								  escolhaDeAlgoritmoAleatoria);
		}
	}

	return distancia;
}

double TSPAction::DoubleBridge_Pertubation(std::vector<int>& solucao,
								double distancia) {
	std::vector<int> copiaDaSolucao;
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

void TSPAction::Limitar_Variacoes_Dos_Indices(int& indiceInicial,
								   int& indiceFinal) {
	if (indiceFinal < indiceInicial) {
		int aux = indiceFinal;
		indiceFinal = indiceInicial;
		indiceInicial = aux;
	}

	int limiteMaximo = std::min(15, dimension / 4);

	if ((indiceFinal - indiceInicial) > limiteMaximo)
		indiceFinal = indiceInicial + limiteMaximo;
}

void TSPAction::VerificarLimitesParaIndices(int& indiceInicial1,
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

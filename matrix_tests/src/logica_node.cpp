#include "logica_node.h"

/* class DiaGraph {
    // insert new nodes into adjacency list from given graph
    adjNode* getAdjListNode(int value, adjNode* head) {
        adjNode* newNode = new adjNode;
        newNode->val = value;
        newNode->next = head;   // point new node to current head
        return newNode;
    }
    int N;  // number of nodes in the graph
public:
    adjNode** head; //adjacency list as array of pointers

    // Constructor
    DiaGraph(std::vector <graphEdge> edges, int n, int N) {
        // allocate new node
        head = new adjNode * [N]();
        this->N = N;
        // initialize head pointer for all vertices
        for (int i = 0; i < N; ++i)
            head[i] = nullptr;
        // construct directed graph by adding edges to it
        for (int i = 0; i < n; i++) {
            int start_ver = edges[i].start_ver;
            int end_ver = edges[i].end_ver;

            // insert in the beginning
            adjNode* newNode = getAdjListNode(end_ver, head[start_ver]);

            // point head pointer to new node
            head[start_ver] = newNode;
        }
    }

    // Destructor
    ~DiaGraph() {
        for (int i = 0; i < N; i++)
            delete[] head[i];
        delete[] head;
    }
};

// print all adjacent vertices of given vertex
void display_AdjList(adjNode* ptr, int i)
{
    while (ptr != nullptr) {
        cout << "(" << i << ", " << ptr->val
            << ") ";
        ptr = ptr->next;
    }
    cout << endl;
}

void print_Vector(std::vector< int > arr)
{
    //print vector
    for (int i = 0; i < arr.size(); i++) {
        std::cout << arr.at(i) << ' ';
    }

    cout << endl;
}

void sort_Vector(std::vector< int > &arr)
{
    for (int i = 0; i < arr.size() - 1; i++) {
        if (arr[i] >= arr[i + 1]) {
            swap(arr[i], arr[i + 1]);
        }
    }
}

// give token to the first vertices
void give_InitToken(adjNode** head, std::vector <blocks> blocks, int N, std::vector <int> &arr)
{
    // create vector with all vertices
    for (int i = 0; i < N; i++)
    {
        arr.push_back(blocks[i].id);
    }

    // remove vertices that arent initials
    for (int i = 0; i < N; i++) 
    {
        adjNode* ptr = head[i];

        while (ptr != nullptr) {
            arr.erase(std::remove(arr.begin(), arr.end(), ptr->val), arr.end());
            ptr = ptr->next;
        }
    }
    
    // remove duplicate values from vector
    auto end = arr.end();
    for (auto it = arr.begin(); it != end; ++it) {
        end = std::remove(it + 1, end, *it);
    }
    arr.erase(end, arr.end());
}



void check_Token(adjNode** head, int N, std::vector< int > &arr, std::vector <blocks> blocks)
{
    std::vector< int > add; // tokens a adicionar no fim
    std::vector< int > rem; // tokens a remover no fim
    
    // search in heap
    for (int i = 0; i < N; i++)
    {
        if (!(std::find(arr.begin(), arr.end(), blocks[i].id) != arr.end())) // verifica apenas os sem token
        { 
            std::vector< int > inputs; // vector para guardar os inputs do bloco
            bool timers_done = false;

            // guarda no array todos os inputs necessarios
            for (int j = 0; j < N; j++)
            {
                adjNode* ptr = head[j];

                while (ptr != nullptr)
                {
                    if (ptr->val == blocks[i].id)
                    {
                        inputs.push_back(j);
                    }
                    ptr = ptr->next;
                }
            }

            // verifica se todos os inputs tem token
            int count = 0;
            for (int k = 0; k < inputs.size(); k++) 
            {
                for (int l = 0; l < arr.size(); l++) 
                {
                    if (inputs[k] == arr[l])
                    {
                        count++;
                    }
                }
            }

            // verifica eventuais timers
            int num_timers = 0;
            for (int j = 0; j < N; j++)
            {
                if (std::find(inputs.begin(), inputs.end(), blocks[j].id) != inputs.end())
                {
                    if (blocks[j].type == "timer")
                    {
                        num_timers++;
                        if (blocks[j].time_done)
                        {
                            timers_done = true;
                        }
                        else
                        {
                            timers_done = false;
                        }
                    }
                }
            }
            if (num_timers == 0) timers_done = true;
            
            if (count == inputs.size() && inputs.size() > 0 && timers_done)
            {
                add.push_back(blocks[i].id);
                for (int k = 0; k < inputs.size(); k++)
                {
                    rem.push_back(inputs[k]);
                }
            }
        }
    }
    
    //atualizar array com os token
    for (int i = 0; i < rem.size(); i++)
    {
        arr.erase(std::remove(arr.begin(), arr.end(), rem[i]), arr.end());
    }
    for (int i = 0; i < add.size(); i++)
    {
        arr.push_back(add[i]);
    }

}

void construct_Blocks(std::vector <blocks> &arr, std::vector <int> id, int N) // inicializa vetor com todos os blocos
{
    for (int i = 0; i < N; i++)
    {
        blocks newBlock;
        newBlock.id = i;
        newBlock.class_id = id[i];
        newBlock.out_f = NULL;
        newBlock.out_b = NULL;
        newBlock.constant = "";
        newBlock.timer = NULL;
        newBlock.time_done = false;

        switch (id[i])
        {
            case 0:
                newBlock.type = "operator";
                newBlock.name = "sum";
                break;
            case 1:
                newBlock.type = "operator";
                newBlock.name = "product";
                break;
            case 2:
                newBlock.type = "operator";
                newBlock.name = "simetric";
                break;
            case 27:
                newBlock.type = "choice";
                newBlock.name = "mux";
                break;
            case 3:
                newBlock.type = "choice";
                newBlock.name = "if";
                break;
            case 32:
                newBlock.type = "choice";
                newBlock.name = "else_if";
                break;
            case 25:
                newBlock.type = "choice";
                newBlock.name = "and";
                break;
            case 26:
                newBlock.type = "choice";
                newBlock.name = "or";
                break;
            case 4:
                newBlock.type = "comparator";
                newBlock.name = "less";
                break;
            case 5:
                newBlock.type = "comparator";
                newBlock.name = "greater";
                break;
            case 6:
                newBlock.type = "comparator";
                newBlock.name = "equal";
                break;
            case 7:
                newBlock.type = "motor";
                newBlock.name = "m_left";
                break;
            case 8:
                newBlock.type = "motor";
                newBlock.name = "m_right";
                break;
            case 9:
                newBlock.type = "sensor";
                newBlock.name = "s_left";
                break;
            case 11:
                newBlock.type = "sensor";
                newBlock.name = "s_front";
                break;
            case 10:
                newBlock.type = "sensor";
                newBlock.name = "s_right";
                break;
            case 12:
                newBlock.type = "sensor";
                newBlock.name = "s_back";
                break;
            case 14:
                newBlock.type = "constant";
                newBlock.name = "zero";
                break;
            case 15:
                newBlock.type = "constant";
                newBlock.name = "one";
                break;
            case 16:
                newBlock.type = "constant";
                newBlock.name = "two";
                break;
            case 17:
                newBlock.type = "constant";
                newBlock.name = "three";
                break;
            case 18:
                newBlock.type = "constant";
                newBlock.name = "four";
                break;
            case 19:
                newBlock.type = "constant";
                newBlock.name = "five";
                break;
            case 20:
                newBlock.type = "constant";
                newBlock.name = "six";
                break;
            case 21:
                newBlock.type = "constant";
                newBlock.name = "seven";
                break;
            case 22:
                newBlock.type = "constant";
                newBlock.name = "eight";
                break;
            case 23:
                newBlock.type = "constant";
                newBlock.name = "nine";
                break;
            case 24:
                newBlock.type = "constant";
                newBlock.name = "comma";
                break;
            case 13:
                newBlock.type = "timer";
                newBlock.name = "delay";
                break;
            case 34:
                newBlock.type = "transport";
                newBlock.name = "t_left";
                break;
            case 35:
                newBlock.type = "transport";
                newBlock.name = "t_right";
                break;
            case 33:
                newBlock.type = "extend";
                newBlock.name = "duplicate";
                break;
            default:
                break;
        }

        arr.push_back(newBlock);
    }
}

void update_BlocksOutputs(adjNode** head, std::vector <blocks> &blocks, std::vector< int > tokens, std::vector <graphEdge> edges, 
    float out_vel[2], float real_vel[2], float sensors[4])
{
    //avaliar os vertices com token
    for (int i = 0; i < blocks.size(); i++)
    {
        if (std::find(tokens.begin(), tokens.end(), blocks[i].id) != tokens.end()) // verifica apenas os com token
        {
            std::vector< int > inputs; // vector para guardar os inputs do bloco

            // guarda no array todos os inputs necessarios
            for (int j = 0; j < blocks.size(); j++)
            {
                adjNode* ptr = head[j];

                while (ptr != nullptr)
                {
                    if (ptr->val == blocks[i].id)
                    {
                        inputs.push_back(j);
                    }
                    ptr = ptr->next;
                }
            }

            if (inputs.size() == 0) // initial token (motor, sensor or constant)
            {
                if (blocks[i].type == "constant")
                {
                    blocks[i].out_f = blocks[i].class_id - (float) 14; // 14 is the class_id of zero
                    blocks[i].constant.push_back(int(blocks[i].out_f) + 48); // ASCII
                }
                else if (blocks[i].type == "sensor")
                {
                    blocks[i].out_f = sensors[blocks[i].class_id - 9]; // 9 is the class_id of s_left
                }
                else if (blocks[i].type == "motor")
                {
                    blocks[i].out_f = real_vel[blocks[i].class_id - 7]; // 7 is the class_id of m_left
                }
            }

            else // sequential token, need to check previous blocks outputs
            {
                int p1, p2, pc; //var. auxiliar para guardar id's dos blocos
                p1 = p2 = pc = NULL;

                float f1, f2; //var. auxiliar para guardar output (float)
                bool b1, b2, bc; //var. auxiliar para guardar output (bool)
                string ct = ""; //var. auxiliar para guardar constant (string)
                f1 = f2 = NULL;
                b1 = b2 = bc = NULL;

                for (int j = 0; j < inputs.size(); j++) //guarda os id's dos inputs na respetiva posicao
                {
                    for (int k = 0; k < edges.size(); k++)
                    {
                        if ((edges[k].start_ver == inputs[j]) && (edges[k].end_ver == blocks[i].id)) // procura edge para os inputs
                        {
                            if (edges[k].input_id == 1) p1 = inputs[j];
                            else if (edges[k].input_id == 2) p2 = inputs[j];
                            else if (edges[k].input_id == 3) pc = inputs[j];
                        }
                    }
                }
                    
                for (int j = 0; j < blocks.size(); j++) // guardar outputs anteriores em var. auxiliares
                {
                    if (blocks[j].id == p1)
                    {
                        f1 = blocks[j].out_f;
                        b1 = blocks[j].out_b;
                        if (blocks[j].constant != "") { ct = blocks[j].constant; }
                    }
                    else if (blocks[j].id == p2)
                    {
                        f2 = blocks[j].out_f;
                        b2 = blocks[j].out_b;
                    }
                    else if (blocks[j].id == pc)
                    {
                        bc = blocks[j].out_b;
                    }
                }

                // tendo em conta o tipo do bloco atual, e os outputs anteriores, atribuir um output ao bloco atual 
                if (blocks[i].name == "sum")
                {
                    blocks[i].out_f = f1 + f2;
                }
                else if (blocks[i].name == "product")
                {
                    blocks[i].out_f = f1 * f2;
                }
                else if (blocks[i].name == "simetric")
                {
                    blocks[i].out_f = -f1;
                }
                else if (blocks[i].name == "mux")
                {
                    if (bc)
                    {
                        blocks[i].out_f = f1;
                        blocks[i].out_b = b1;
                    }
                    else
                    {
                        blocks[i].out_f = f2;
                        blocks[i].out_b = b2;
                    }  
                }
                else if (blocks[i].name == "if")
                {
                    if (bc)
                    {
                        blocks[i].out_f = f1;
                        blocks[i].out_b = b1;
                    }
                }
                else if (blocks[i].name == "else_if")
                {
                    if (bc)
                    {
                        blocks[i].out_f = f1;
                        blocks[i].out_b = b1;
                    }
                    else
                    {
                        blocks[i].out_f = f2;
                        blocks[i].out_b = b2;
                    }
                }
                else if (blocks[i].name == "and")
                {
                    blocks[i].out_b = b1 && b2;
                }
                else if (blocks[i].name == "or")
                {
                    blocks[i].out_b = b1 || b2;
                }
                else if (blocks[i].name == "less")
                {
                    blocks[i].out_b = (f1 < f2);
                }
                else if (blocks[i].name == "greater")
                {
                    blocks[i].out_b = (f1 > f2);
                }
                else if (blocks[i].name == "equal")
                {
                    blocks[i].out_b = (f1 == f2);
                }
                else if (blocks[i].name == "m_left")
                {
                    out_vel[0] = f1;
                    blocks[i].out_f = real_vel[0];
                }
                else if (blocks[i].name == "m_right")
                {
                    out_vel[1] = f1;
                    blocks[i].out_f = real_vel[1];
                }
                else if (blocks[i].name == "s_left")
                {
                    blocks[i].out_f = sensors[0];
                }
                else if (blocks[i].name == "s_right")
                {
                    blocks[i].out_f = sensors[1];
                }
                else if (blocks[i].name == "s_front")
                {
                    blocks[i].out_f = sensors[2];
                }
                else if (blocks[i].name == "s_back")
                {
                    blocks[i].out_f = sensors[3];
                }
                else if (blocks[i].name == "comma")
                {
                    ct = ct.append(".");
                    blocks[i].constant = ct;
                    blocks[i].out_f = f1;
                }
                else if (blocks[i].type == "constant" && !(blocks[i].name == "comma"))
                {
                    int num = blocks[i].class_id - 14; // 14 is the class_id of zero
                    ct.push_back(num+48); // ASCII
                    blocks[i].constant = ct;
                    blocks[i].out_f = strtof(ct.c_str(), NULL);
                }
                else if (blocks[i].name == "delay")
                {
                    if (blocks[i].timer == NULL)
                    {
                        blocks[i].timer = clock() + (CLOCKS_PER_SEC * f2);
                    }
                }
            }
        }
    }
}

void stopActionTimer(adjNode** head, std::vector <blocks> &blocks, std::vector< int > tokens, float vel[2])
{
    for (int i = 0; i < blocks.size(); i++)
    {
        if ((std::find(tokens.begin(), tokens.end(), blocks[i].id) != tokens.end()) && (blocks[i].type == "timer")) // verifica apenas os com token
        {
            std::vector< int > inputs; // vector para guardar os inputs do bloco

            // guarda no array todos os inputs necessarios
            for (int j = 0; j < blocks.size(); j++)
            {
                adjNode* ptr = head[j];

                while (ptr != nullptr)
                {
                    if (ptr->val == blocks[i].id)
                    {
                        inputs.push_back(j);
                    }
                    ptr = ptr->next;
                }
            }
            
            for (int j = 0; j < blocks.size(); j++)
            {
                for (int k = 0; k < inputs.size(); k++)
                {
                    if (blocks[j].name == "m_left")
                    {
                        vel[0] = 0;
                    }
                    if (blocks[j].name == "m_right")
                    {
                        vel[1] = 0;
                    }
                }
            }
        }
    }
}

void updateTimers(adjNode** head, std::vector <blocks>& blocks, std::vector< int > tokens, float vel[2])
{
    for (int i = 0; i < blocks.size(); i++)
    {
        if ((std::find(tokens.begin(), tokens.end(), blocks[i].id) != tokens.end()) && (blocks[i].type == "timer")) // verifica apenas os com token
        {
            if ((clock() >= blocks[i].timer) && (blocks[i].time_done == false))
            {
                blocks[i].time_done = true;
                stopActionTimer(head, blocks, tokens, vel);
            }
        }
    }
}

bool checkIfChanged(std::vector <int> tokens_previous, std::vector <int> tokens_actual)
{
    sort_Vector(tokens_previous);
    sort_Vector(tokens_actual);
    print_Vector(tokens_previous);
    print_Vector(tokens_actual);

    if (tokens_previous == tokens_actual)
    {
        cout << "EQUAL" << endl;
        return true;
    }

    return false;
}

int check_EdgesLogic(std::vector <graphEdge> edges, std::vector <blocks> arr, int n) // criar func para verificar se todas as ligacoes fazem sentido 
{
    for (int i = 0; i < n; i++) 
    {
        string start_type = "";
        string end_type = "";
        string start_name = "";
        string end_name = "";

        for (int j = 0; j < arr.size(); j++)
        {
            if (arr[j].id == edges[i].start_ver)
            {
                start_type = arr[j].type;
                start_name = arr[j].name;
            }
            if (arr[j].id == edges[i].end_ver)
            {
                end_type = arr[j].type;
                end_name = arr[j].name;
            }
        }

        if (start_type == "operator")
        {
            if ((end_type == "choice") && (edges[i].input_id == 3))
            {
                return -1;
            }
            else if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
            }
            else if ((end_type == "timer") && (edges[i].input_id == 1)) 
            {
                return -1;
            }
        }

        if (start_type == "choice")
        {
            if ((start_name == "or") || (start_name == "and"))
            {
                if (end_type == "operator")
                {
                    if ((end_name == "sum") || (end_name == "product"))
                    {
                        return -1;
                    }
                }
                else if (end_type == "choice")
                {
                    if ((edges[i].input_id == 1) || (edges[i].input_id == 2))
                    {
                        return -1;
                    }
                }
                else if ((end_type == "comparator") || (end_type == "motor") || 
                    (end_type == "constant") || (end_type == "timer"))
                {
                    return -1;
                }
            }
            else //mux, if, else if - float
            {
                if (end_type == "choice")
                {
                    if (edges[i].input_id == 3)
                    {
                        return -1;
                    }
                    else if ((end_name == "or") || (end_name == "and"))
                    {
                        return -1;
                    }
                }
                else if ((end_type == "timer") && (edges[i].input_id == 1))
                {
                    return -1;
                }
            }
        }

        if (start_type == "comparator")
        {
            if ((end_type == "operator") && (end_name != "simetric"))
            {
                return -1;
            }
            else if ((end_type == "choice") && (edges[i].input_id != 3))
            {
                return -1;
            }
            else if ((end_type == "comparator") || (end_type == "motor") || 
                (end_type == "constant") || (end_type == "timer"))
            {
                return -1;
            }
        }

        if ((start_type == "motor") || (start_type == "sensor") || 
            (start_type == "constant"))
        {
            if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
                else if (edges[i].input_id == 3)
                {
                    return -1;
                }
            }
            else if (end_type == "comparator")
            {
                return -1;
            }
            
        }

        if ((start_type == "sensor") || (start_type == "constant"))
        {
            if ((end_type == "timer") && (edges[i].input_id == 1))
            {
                return -1;
            }
        }
     
        if (start_type == "timer")
        {
            if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
                else if (edges[i].input_id == 3)
                {
                    return -1;
                }
            }
            else if (end_type == "timer")
            {
                return -1;
            }
        }    
    }

    return 0;
}

// verifica se faltam ligacoes e 
// retorna -2 em caso de erro, 0 em caso de sucesso
int check_EdgesQuantity(std::vector <graphEdge> edges, std::vector <blocks> blocks)
{
    // verificar se todos os i/o tem conexoes, erros por defeito ou excesso
    for (int i = 0; i < blocks.size(); i++)
    {
        int n_i1, n_i2, n_cond;
        n_i1 = n_i2 = n_cond = 0;
    
        for (int j = 0; j < edges.size(); j++)
        {
            if (edges[j].end_ver == blocks[i].id) //verificar inputs
            {
                if (edges[j].input_id == 1)
                    n_i1++;
                else if (edges[j].input_id == 2)
                    n_i2++;
                else
                    n_cond++;
            }
        }

        // 0i - 1o: sensores
        if (blocks[i].type == "sensor")
        {
            if (((n_i1 + n_i2 + n_cond) > 0))
                return -2;
        }
        // 1i - 1o: constants
        else if ((blocks[i].type == "constant") || (blocks[i].type == "motor") || (blocks[i].name == "simetric"))
        {
            if ((n_i2 + n_cond) > 0)
                return -2;
        }
        // 1i - 1o: simetrico, motores
        else if ((blocks[i].type == "motor") || (blocks[i].name == "simetric"))
        {
            if ((n_i1 != 1) || ((n_i2 + n_cond) > 0))
                return -2;
        }
        // 2i - 1o: operators (sum & product), comparators, timer, and, or
        else if ((blocks[i].name == "sum") || (blocks[i].name == "product") || (blocks[i].type == "comparator") ||
            (blocks[i].name == "delay") || (blocks[i].name == "and") || (blocks[i].name == "or"))
        {
            if ((n_i1 != 1) || (n_i2 != 1) || (n_cond > 0))
                return -2;
        }
        // 2i (cond) - 1o: if
        else if (blocks[i].name == "if")
        {
            if ((n_i1 != 1) || (n_i2 != 0) || (n_cond != 1))
                return -2;
        }
        // 3i (cond) - 1o: mux e else if
        else if ((blocks[i].name == "mux") || (blocks[i].name == "else_if"))
        {
            if ((n_i1 != 1) || (n_i2 != 1) || (n_cond != 1))
                return -2;
        }
        // 2i - 0o: TE
        else if (blocks[i].name == "t_left")
        {
            if ((n_i1 + n_i2 == 0) || (n_cond != 0))
                return -2;
        }
        // 0i - 2o: TD
        else if (blocks[i].name == "t_right")
        {
            if ((n_i1 + n_i2 + n_cond != 0))
                return -2;
        }
        // 1i - 2o: Extensor
        else if (blocks[i].name == "double")
        {
            if ((n_i1 != 1) || ((n_i2 + n_cond) != 0))
                return -2;
        }
    }
    
    return 0;
}

// verifica ligacoes confusas (i->i, i->o, o->o)
// retorna -3 em caso de erro, 0 em caso de sucesso
int check_EdgesStupid(std::vector< std::vector<int>> matrix, int n)
{
    // procurar ligacoes confusas 
    for (int i = 0; i < n; i++)
    {
        if ((matrix[i][0] == -1) && (matrix[i][1] == -1)) // inputs todos a -1
        {
            return -3;
        }
        else if ((matrix[i][2] == -1) && (matrix[i][3] == -1) && (matrix[i][4] == -1)) // outputs todos a -1
        {
            return -3;
        }
    }

    return 0;
}

// check if the blocks that start the graph makes sense
// retorna -4 em caso de erro, 0 em caso de sucesso
int check_InitialBlocks(std::vector <int> tokens, std::vector <blocks> blocks)
{
    for (int i = 0; i < tokens.size(); i++)
    {
        for (int j = 0; j < blocks.size(); j++)
        {
            if (tokens[i] == blocks[j].id)
            {
                if ((blocks[j].type == "operator") || (blocks[j].type == "choice") || (blocks[j].type == "comparator") ||
                    (blocks[j].type == "timer") || (blocks[j].type == "transport") || (blocks[j].type == "extend") ||
                    (blocks[j].name == "comma"))
                {
                    return -4;
                }
            }
        }
    }

    return 0;
}

// criar funcao para saltar ligacoes quer do bloco transporte quer do bloco extensor
// aplicar antes de criar o diagraph, e alterar o array das edges
void subs_Transport (std::vector <graphEdge> &edges, std::vector <blocks> &arr, int &n, int &N)
{
    bool rem_blocks = false;

    for (int i = 0; i < N; i++) // search blocks
    {
        if (arr[i].name == "t_left")
        {
            for (int j = 0; j < N; j++)
            {
                if (arr[j].name == "t_right")
                {
                    for (int k = 0; k < n; k++) // search edges
                    {
                        if (edges[k].end_ver == arr[i].id)
                        {
                            for (int l = 0; l < n; l++)
                            {
                                if ((edges[l].start_ver == arr[j].id) && (edges[l].output_id == edges[k].input_id))
                                {
                                    graphEdge edge;
                                    edge = {edges[k].start_ver, edges[l].end_ver, edges[k].output_id, edges[l].input_id};
                                    
                                    //remove 2 edges, add 1
                                    edges.erase(edges.begin() + k);
                                    if (l < k)
                                    {
                                        edges.erase(edges.begin() + l);
                                    }
                                    else
                                    {
                                        edges.erase(edges.begin() + l - 1);
                                    }
                                    edges.push_back(edge);
                                    k = 0;
                                    l = 0;
                                    n--;
                                    rem_blocks = true;  
                                }
                            }
                        }
                    }
                    
                    //remove 2 blocks
                    if (rem_blocks)
                    {
                        arr.erase(arr.begin() + i);
                        if (j < i)
                        {
                            arr.erase(arr.begin() + j);
                        }
                        else
                        {
                            arr.erase(arr.begin() + j - 1);
                        }
                        N -= 2;
                        rem_blocks = false;
                    }

                }
            }
        }
    }
}

void subs_Extend(std::vector <graphEdge> &edges, std::vector <blocks> &arr, int &n, int &N)
{
    bool rem_blocks = false;
    int count = 0;

    for (int i = 0; i < N; i++) // search block extend
    {
        if (arr[i].name == "duplicate")
        {
            for (int k = 0; k < n; k++) // search edges
            {
                if (edges[k].end_ver == arr[i].id)
                {
                    for (int l = 0; l < n; l++)
                    {
                        if (edges[l].start_ver == arr[i].id)
                        {
                            graphEdge edge;
                            edge = { edges[k].start_ver, edges[l].end_ver, edges[k].output_id, edges[l].input_id };

                            // remove and add 1st edge
                            if (count == 0)
                            {
                                edges.erase(edges.begin() + l);
                                edges.push_back(edge);
                                k = l = 0;
                                count++;
                            }
                            else // remove 2 edges and add 2nd edge
                            {
                                edges.erase(edges.begin() + k);
                                if (l < k)
                                {
                                    edges.erase(edges.begin() + l);
                                }
                                else
                                {
                                    edges.erase(edges.begin() + l - 1);
                                }
                                edges.push_back(edge);
                                count = 0;
                                rem_blocks = true;
                                n--;
                            }
                        }
                    }
                }
            }

            //remove block
            if (rem_blocks)
            {
                arr.erase(arr.begin() + i);
                N--;
                rem_blocks = false;
            }
        }
    }
}

std::vector< std::vector<int>> arrayToMatrix(int array [], int len)
{
    // array to matrix
    int n = 5;
    int m = len / n;

    std::vector< std::vector<int>> matrix;


    matrix.resize(m); //Grow rows by m
    for (int i = 0; i < m; ++i)
    {
        //Grow Columns by n
        matrix[i].resize(n);
    }

    for (int i = 0; i < len; i++)
    {
        matrix[i / 5][i % 5] = array[i];
    }

    // print matrix
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cout << matrix[i][j] << " ";
        }
        cout << endl;
    }

    return matrix;
}

void matrix_ToEdges(std::vector< std::vector<int>> matrix, std::vector <graphEdge>& edges, int n)
{    
    // matrix to edges
    for (int i = 0; i < n; i++)
    {
        int start, end, inp, out;
        start = end = inp = out = 0;

        if (matrix[i][0] != -1)
        {
            out = 1;
            start = matrix[i][0];
        }
        else
        {
            out = 2;
            start = matrix[i][1];
        }

        if (matrix[i][2] != -1)
        {
            inp = 1;
            end = matrix[i][2];
        }
        else if (matrix[i][3] != -1)
        {
            inp = 2;
            end = matrix[i][3];
        }
        else
        {
            inp = 3;
            end = matrix[i][4];
        }

        graphEdge gE = { start, end, out, inp };
        edges.push_back(gE);
    }
} */

void cbMatrix(const serp::Matrix::ConstPtr &msg){
  if(msg->manual_mode){
    out_vel[0]= msg->vel_motor_left;
    out_vel[1] = msg->vel_motor_right;
    //ROS_INFO("AQUI");
  }
  else{

  }
  return;
}

/* void cbSensors(const serp::ObjectDetection::ConstPtr &msg){
    left = msg->left;
    right = msg->right;
    front = msg->front;
    back = msg->back;
    return;
} */

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "logica_node");
  
  ros::NodeHandle node;
  
  ros::Subscriber matrix_sub = node.subscribe("/matrix", 1, cbMatrix);
  //ros::Subscriber sensors_sub = node.subscribe("/sensors", 1, cbSensors);

  pub_vel = node.advertise<serp::Velocity>("/vel", 1);
  serp::Velocity vel;

  while(ros::ok()){
    vel.vel_motor_left = out_vel[0];
    vel.vel_motor_right = out_vel[1];
    pub_vel.publish(vel);
    ros::spinOnce();
  }

  ros::spin();
  return 0;
}

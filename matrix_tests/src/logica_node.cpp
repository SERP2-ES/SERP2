#include "logica_node.h"

bool checkIfChanged(std::vector <int> tokens_previous)
{
    std::sort(tokens_previous.begin(), tokens_previous.end());
    std::sort(tokens.begin(), tokens.end());

    if (tokens_previous == tokens)
    {
        return false;
    }

    return true;
}

void update_BlocksOutputs()
{
    //avaliar os vertices com token
    for (int i = 0; i < blocks_list.size(); i++)
    {
        if (std::find(tokens.begin(), tokens.end(), blocks_list[i].id) != tokens.end()) // verifica apenas os com token
        {
            std::vector< int > inputs; // vector para guardar os inputs do bloco
            
             // guarda no array todos os inputs necessarios
            for (int j = 0; j < edges.size(); j++)
            {
                if(edges.at(j).end_ver == blocks_list.at(i).id){
                    inputs.push_back(edges.at(j).start_ver);
                }
            }
           

            if (inputs.size() == 0) // initial token (motor, sensor or constant)
            {
                if (blocks_list[i].type == "constant")
                {
                    blocks_list[i].out_f = float(blocks_list[i].class_id) - (float) 14; // 14 is the class_id of zero
                    blocks_list[i].constant.push_back(int(blocks_list[i].out_f) + 48); // ASCII
                }
                else if (blocks_list[i].type == "sensor")
                {
                    blocks_list[i].out_f = sensors[blocks_list[i].class_id - 9]; // 9 is the class_id of s_left
                }
            }

            else // sequential token, need to check previous blocks outputs
            {
                int p1, p2, pc; //var. auxiliar para guardar id's dos blocos
                p1 = p2 = pc = -1;

                float f1, f2; //var. auxiliar para guardar output (float)
                bool b1, b2, bc; //var. auxiliar para guardar output (bool)
                std::string ct = ""; //var. auxiliar para guardar constant (string)
                f1 = f2 = NAN;
                b1 = b2 = bc = NULL;

                for (int j = 0; j < inputs.size(); j++) //guarda os id's dos inputs na respetiva posicao
                {
                    for (int k = 0; k < edges.size(); k++)
                    {
                        if ((edges[k].start_ver == inputs[j]) && (edges[k].end_ver == blocks_list[i].id)) // procura edge para os inputs
                        {
                            if (edges[k].input_id == 1) p1 = inputs[j];
                            else if (edges[k].input_id == 2) p2 = inputs[j];
                            else if (edges[k].input_id == 3) pc = inputs[j];
                        }
                    }
                }
                    
                for (int j = 0; j < blocks_list.size(); j++) // guardar outputs anteriores em var. auxiliares
                {
                    if (blocks_list[j].id == p1)
                    {
                        f1 = blocks_list[j].out_f;
                        b1 = blocks_list[j].out_b;
                        if (blocks_list[j].constant != "") { ct = blocks_list[j].constant; }
                    }
                    else if (blocks_list[j].id == p2)
                    {
                        f2 = blocks_list[j].out_f;
                        b2 = blocks_list[j].out_b;
                    }
                    else if (blocks_list[j].id == pc)
                    {
                        bc = blocks_list[j].out_b;
                    }
                }

                // tendo em conta o tipo do bloco atual, e os outputs anteriores, atribuir um output ao bloco atual 
                if (blocks_list[i].name == "sum")
                {
                    blocks_list[i].out_f = f1 + f2;
                }
                else if (blocks_list[i].name == "product")
                {
                    blocks_list[i].out_f = f1 * f2;
                }
                else if (blocks_list[i].name == "simetric")
                {
                    blocks_list[i].out_f = -f1;
                }
                else if (blocks_list[i].name == "mux")
                {
                    if (bc)
                    {
                        blocks_list[i].out_f = f1;
                        blocks_list[i].out_b = b1;
                    }
                    else
                    {
                        blocks_list[i].out_f = f2;
                        blocks_list[i].out_b = b2;
                    }  
                }
                else if (blocks_list[i].name == "if")
                {
                    if (bc)
                    {
                        blocks_list[i].out_f = f1;
                        blocks_list[i].out_b = b1;
                    }
                }
                else if (blocks_list[i].name == "else_if")
                {
                    if (bc)
                    {
                        blocks_list[i].out_f = f1;
                        blocks_list[i].out_b = b1;
                    }
                    else
                    {
                        blocks_list[i].out_f = f2;
                        blocks_list[i].out_b = b2;
                    }
                }
                else if (blocks_list[i].name == "and")
                {
                    blocks_list[i].out_b = b1 && b2;
                }
                else if (blocks_list[i].name == "or")
                {
                    blocks_list[i].out_b = b1 || b2;
                }
                else if (blocks_list[i].name == "less")
                {
                    blocks_list[i].out_b = (f1 < f2);
                }
                else if (blocks_list[i].name == "greater")
                {
                    blocks_list[i].out_b = (f1 > f2);
                }
                else if (blocks_list[i].name == "equal")
                {
                    blocks_list[i].out_b = (f1 == f2);
                }
                else if (blocks_list[i].name == "m_left")
                {
                    out_vel[0] = f1;
                    blocks_list[i].out_f = f1;
                }
                else if (blocks_list[i].name == "m_right")
                {
                    out_vel[1] = f1;
                    blocks_list[i].out_f = f1;
                }
                else if (blocks_list[i].name == "s_left")
                {
                    blocks_list[i].out_f = sensors[0];
                }
                else if (blocks_list[i].name == "s_right")
                {
                    blocks_list[i].out_f = sensors[1];
                }
                else if (blocks_list[i].name == "s_front")
                {
                    blocks_list[i].out_f = sensors[2];
                }
                else if (blocks_list[i].name == "s_back")
                {
                    blocks_list[i].out_f = sensors[3];
                }
                else if (blocks_list[i].name == "comma")
                {
                    ct = ct.append(".");
                    blocks_list[i].constant = ct;
                    blocks_list[i].out_f = f1;
                }
                else if (blocks_list[i].type == "constant" && !(blocks_list[i].name == "comma"))
                {
                    int num = blocks_list[i].class_id - 14; // 14 is the class_id of zero
                    ct.push_back(num+48); // ASCII
                    blocks_list[i].constant = ct;
                    blocks_list[i].out_f = strtof(ct.c_str(), NULL);
                }
                else if (blocks_list[i].name == "delay")
                {
                    if (blocks_list[i].timer == NULL)
                    {
                        blocks_list[i].timer = clock() + (CLOCKS_PER_SEC * f2);
                    }
                }
            }
        }
    }
}

void cleanVariables(){
    edges.clear();
    blocks_list.clear();
    tokens.clear();
    num_block = num_rows = num_col = 0;
}

void check_Token()
{
    std::vector< int > add; // tokens a adicionar no fim
    std::vector< int > rem; // tokens a remover no fim
    
    // search in heap
    for (int i = 0; i < num_block; i++)
    {   
        if (!(std::find(tokens.begin(), tokens.end(), blocks_list.at(i).id) != tokens.end())) // verifica apenas os sem token
        { 
            std::vector< int > inputs; // vector para guardar os inputs do bloco
            bool timers_done = false;

            // guarda no array todos os inputs necessarios
            for (int j = 0; j < edges.size(); j++)
            {
                if(edges.at(j).end_ver == blocks_list.at(i).id){
                    inputs.push_back(edges.at(j).start_ver);
                }
            }

            // verifica se todos os inputs tem token
            int count = 0;
            for (int k = 0; k < inputs.size(); k++) 
            {
                for (int l = 0; l < tokens.size(); l++) 
                {
                    if (inputs[k] == tokens[l])
                    {
                        count++;
                    }
                }
            }

            // verifica eventuais timers
            int num_timers = 0;
            for (int j = 0; j < num_block; j++)
            {
                if (std::find(inputs.begin(), inputs.end(), blocks_list[j].id) != inputs.end())
                {
                    if (blocks_list[j].type == "timer")
                    {
                        num_timers++;
                        if (blocks_list[j].time_done)
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
                add.push_back(blocks_list[i].id);
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
        tokens.erase(std::remove(tokens.begin(), tokens.end(), rem[i]), tokens.end());
    }
    for (int i = 0; i < add.size(); i++)
    {
        tokens.push_back(add[i]);
    }

}

void give_InitToken()
{
    // create vector with all vertices
    for (int i = 0; i < num_block; i++)
    {
        tokens.push_back(blocks_list[i].id);
    }

    // remove vertices that arent initials
    for (int i = 0; i < edges.size(); i++) 
    {
        tokens.erase(std::remove(tokens.begin(), tokens.end(), edges.at(i).end_ver), tokens.end());
    }
    
    // remove duplicate values from vector
    auto end = tokens.end();
    for (auto it = tokens.begin(); it != end; ++it) {
        end = std::remove(it + 1, end, *it);
    }
    tokens.erase(end, tokens.end());
    copy(tokens.begin(), tokens.end(), back_inserter(init));
}

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
                    (blocks[j].name == "comma") ||
                    (blocks[j].type == "motor"))
                {
                    return -4;
                }
            }
        }
    }

    return 0;
}

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

int check_EdgesQuantity(std::vector <graphEdge> edges, std::vector <blocks> blocks)
{
    // verificar se todos os i/o tem conexoes, erros por defeito ou excesso
    for (int i = 0; i < blocks.size(); i++)
    {
        int n_i1, n_i2, n_cond, out_1, out_2;
        n_i1 = n_i2 = n_cond = out_1 = out_2 = 0;
    
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
            if (edges[j].start_ver == blocks[i].id) //verificar outputs
            {
                if (edges[j].output_id == 1)
                    out_1++;
                else if (edges[j].output_id == 2)
                    out_2++;
            }
        }

        // blocks without any edge linked
        if ((n_i1 + n_i2 + n_cond + out_1 + out_2) <= 0)
        {
            return -2;
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

int check_EdgesLogic() // criar func para verificar se todas as ligacoes fazem sentido 
{
    for (int i = 0; i < num_rows; i++) 
    {
        std::string start_type = "";
        std::string end_type = "";
        std::string start_name = "";
        std::string end_name = "";

        for (int j = 0; j < blocks_list.size(); j++)
        {
            if (blocks_list[j].id == edges[i].start_ver)
            {
                start_type = blocks_list[j].type;
                start_name = blocks_list[j].name;
            }
            if (blocks_list[j].id == edges[i].end_ver)
            {
                end_type = blocks_list[j].type;
                end_name = blocks_list[j].name;
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

void matrix_ToEdges(std::vector< std::vector<int>> matrix)
{    
    // matrix to edges
    for (int i = 0; i < num_rows; i++)
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
} 

void construct_Blocks(std::vector <int> id, int N) // inicializa vetor com todos os blocos
{
    for (int i = 0; i < N; i++)
    {
        blocks newBlock;
        newBlock.id = i+1;
        newBlock.class_id = id[i];
        newBlock.out_f = NAN;
        newBlock.out_b = NAN;
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

        blocks_list.push_back(newBlock);
    }

    // std::cout << "BLOCK_LIST" << std::endl;
    // for(int i = 0; i < blocks_list.size(); i++ ){
    //     std::cout << blocks_list.at(i) << std::endl;
    // }
}

void subs_Extend()
{
    bool rem_blocks = false;

    for (int i = 0; i < num_block; i++) // search block extend
    {   
        int before_ext = -1;
        int before_ext_out = -1;
        int ext_id = -1;
        int after_ext_1 = -1;
        int after_ext_2 = -1;
        int after_ext_1_inp = -1;
        int after_ext_2_inp = -1;
    
        if (blocks_list[i].name == "duplicate")
        {   
            ext_id = blocks_list[i].id;

            for (int k = 0; k < num_rows; k++) // search edges
            {   
                if (edges[k].end_ver == blocks_list[i].id)
                {   
                    before_ext = edges[k].start_ver;
                    before_ext_out = edges[k].output_id;

                    for (int l = 0; l < num_rows; l++)
                    {   
                        if (edges[l].start_ver == blocks_list[i].id)
                        {   
                            if (after_ext_1 == -1)
                            {
                                after_ext_1 = edges[l].end_ver;
                                after_ext_1_inp = edges[l].input_id;
                            }    
                            else 
                            {
                                after_ext_2 = edges[l].end_ver;    
                                after_ext_2_inp = edges[l].input_id;
                            }
                        }
                    }
                }
            }
            
            //remove 3 edges around extensor
            for (int m=0;m<edges.size();m++)
            {
                if ((edges[m].start_ver == ext_id) || (edges[m].end_ver == ext_id))
                {
                    edges.erase(edges.begin() + m);
                    m = -1;
                }
            }

            //add 2 edges 
            graphEdge edge1 = {before_ext, after_ext_1, before_ext_out, after_ext_1_inp};
            graphEdge edge2 = {before_ext, after_ext_2, before_ext_out, after_ext_2_inp};
            edges.push_back(edge1);
            edges.push_back(edge2);
            num_rows--;
            
            //remove block
            blocks_list.erase(blocks_list.begin() + i);
            num_block--;
        }
    }
}

void cbMatrix(const serp::Matrix::ConstPtr &msg){
    ROS_INFO("CALLBACK");
   
    if(msg->manual_mode){
        out_vel[0] = msg->vel_motor_left;
        out_vel[1] = msg->vel_motor_right;
        matrix_rcv = false;
    }
    else{
        matrix_rcv = true; 
        num_block = msg->matrix1.size();
        num_rows = msg->matrix2.size() / 5;
        num_col = 5;
        std::vector<int> matrix1;
        for(int i=0; i < num_block; i++){
            matrix1.push_back(msg->matrix1[i]);
        }

        std::vector<std::vector<int>> matrix2;
        matrix2.resize(num_rows);
        for(int i = 0; i < num_rows; i++ ){
            matrix2[i].resize(num_col);
        }

        for (int i = 0; i < num_rows*num_col; i++)
        {
            matrix2[i / 5][i % 5] = msg->matrix2[i];
        }

        //print matrix
        //std::cout << "Matrix 1" << std::endl;
        //for(int i = 0; i < matrix1.size(); i++ ){
        //    std::cout << matrix1.at(i) << " ";
        //}
        //std::cout << std::endl;
        
        //std::cout << "Matrix 2" << std::endl;
        //for (int i = 0; i < matrix2.size(); i++)
        //{
        //    for (int j = 0; j < matrix2[i].size(); j++)
        //    {
        //        std::cout << matrix2[i][j] << " ";
        //    }
        //    std::cout << std::endl;
        //}

        construct_Blocks(matrix1, num_block);
        matrix_ToEdges(matrix2);
        
        subs_Extend();
        give_InitToken();

        error[0] = check_EdgesLogic();
        error[1] = check_EdgesQuantity(edges, blocks_list);
        error[2] = check_EdgesStupid(matrix2, num_rows);
        error[3] = check_InitialBlocks(tokens, blocks_list);

        for(int i=0; i < 4; i++){
            if(error[i] < 0){
                std_msgs::Int8 e;
                e.data = error[i];
                pub_errors.publish(e);
                return;
            }
        }

        update_BlocksOutputs();
    }
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "logic_node");
  
  ros::NodeHandle node;
  ros::Rate rate(5);
  
  ros::Subscriber matrix_sub = node.subscribe("/matrix", 1, cbMatrix);
  //ros::Subscriber sensors_sub = node.subscribe("/sensors", 1, cbSensors);

  pub_vel = node.advertise<serp::Velocity>("/vel", 1);
  out_vel[0] = 0;
  out_vel[1] = 0;
  
  serp::Velocity vel;
  pub_errors = node.advertise<std_msgs::Int8>("/error_logic", 1);

  while(ros::ok()){
    if(matrix_rcv){
        it++;

        std::vector< int > last_tokens; // vector with last cycle tokens
        copy(tokens.begin(), tokens.end(), back_inserter(last_tokens));

        
        check_Token();
        update_BlocksOutputs();

        if(!checkIfChanged(last_tokens)){
            tokens.clear();
            copy(init.begin(), init.end(), back_inserter(tokens));
        }
    }
    
    vel.vel_motor_left = out_vel[0];
    vel.vel_motor_right = out_vel[1];
    pub_vel.publish(vel);
    ros::spinOnce();
  }

  return 0;
}
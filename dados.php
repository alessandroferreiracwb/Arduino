<?php
if (isset($_POST['data'])) {
    $dados_recebidos = $_POST['data'];
    // Salva os dados em um arquivo de texto
    file_put_contents('dados_k7.txt', $dados_recebidos);
    echo "Dados recebidos com sucesso!";
}

if (isset($_GET['ler_dados'])) {
    // Lê os dados do arquivo
    $dados = file_get_contents('dados_k7.txt');
    echo $dados;
}
?>
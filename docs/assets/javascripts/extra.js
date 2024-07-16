$(document).ready(function () {

    // Set the branch to use for the SUTURO objects source files
    var branch = "robocup";
    // Set the relative path to the owl2anything output folder
    var relativeFolderPath = "owl2anything/output";
    var repoFolderPath = `docs/objects/${relativeFolderPath}`;

    // Initialize SUTURO objects table (if present)
    if ($('#objects-csv-table').length) {
        $.ajax({
            url: `${relativeFolderPath}/suturo_objects.csv`,
            dataType: "text",
            success: function (data) {

                // Split the data into rows
                var rows = data.trim().split(/\r?\n|\r/);

                // Get the headers from the first row
                var headers = rows.shift().split(';');

                // Find the indices of the desired columns
                var nameColumn = headers.indexOf("Name");
                var iriShortColumn = headers.indexOf("IRI ShortForm");
                var perceptionIdColumn = headers.indexOf("Perception Id");
                var descriptionColumn = headers.indexOf("Description");

                // Create an array with the desired column titles
                var headerColumns = [headers[nameColumn], "IRI", "Id", headers[descriptionColumn]];

                // Initialize the DataTable with the desired columns
                var table = $('#objects-csv-table').DataTable({
                    responsive: true,
                    columns: headerColumns.map(function (headerName) {
                        return { title: headerName };
                    })
                });

                rows.forEach(function (row) {
                    var rowData = row.split(';');
                    // Filter the row to include only the desired columns
                    var filteredRow = [rowData[nameColumn], rowData[iriShortColumn], rowData[perceptionIdColumn], rowData[descriptionColumn]];
                    table.row.add(filteredRow);
                });

                table.draw();
            }
        });
    }

    // Set the last update date of the SUTURO objects table (if present)
    if ($('#objects-last-update').length) {
        $.ajax({
            url: `https://api.github.com/repos/SUTURO/suturo_knowledge/commits?sha=${branch}&path=${repoFolderPath}`,
            success: function (result) {
                // Find the last commit that was made by the owl2anything workflow (Always starts with "Updated SUTURO Objects to", followed by the commit hash)
                const commit = result.find(commit => commit.commit.message.match(/Updated SUTURO Objects to [a-f0-9]{7}/));
                // Extract the commit message and display the date and commit hash of the last update
                if (commit) {
                    const commitMessage = commit.commit.message;
                    const commitHash = commitMessage.match(/[a-f0-9]{7}/)[0];
                    const commitDateString = commit.commit.author.date;
                    const commitDate = new Date(commitDateString);
                    const commitDateOptions = { year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false };
                    const commitDateFormatted = commitDate.toLocaleString(undefined, commitDateOptions);
                    const commitLink = `https://github.com/SUTURO/suturo_knowledge/commit/${commitHash}`;

                    const updateText = `Last update: ${commitDateFormatted} (<a href="${commitLink}">#${commitHash}</a>)`;
                    document.getElementById('objects-last-update').innerHTML = updateText;
                }
            }
        });
    }

    // Popultate the object downloads list (if present)
    if ($('#objects-downloads').length) {

        // Array of supported file formats for direct view in browser
        var supportetBrowserFormat = [".txt", ".json"];

        // Use GitHub API to fetch all files from the owl2anything output
        $.ajax({
            url: `https://api.github.com/repos/SUTURO/suturo_knowledge/contents/${repoFolderPath}?ref=${branch}`,
            success: function (data) {

                // Filter and sort the files
                var files = data.filter(function (file) {
                    return file.type === "file";
                }).sort(function (a, b) {
                    return a.name.localeCompare(b.name);
                });

                files.forEach(function (file) {

                    var fileName = file.name;
                    var fileExtension = fileName.substr(fileName.lastIndexOf(".")).toLowerCase();
                    var filePath = relativeFolderPath + "/" + fileName;

                    // Create list item
                    var listItem = $("<li></li>");

                    // Create file name span
                    var fileNameSpan = $("<span></span>").text(fileName + ": ");

                    // Create view button
                    var viewButton = $("<a></a>").text("View");
                    if (supportetBrowserFormat.includes(fileExtension)) {
                        viewButton.attr("href", filePath);
                        viewButton.attr("target", "_blank");
                    } else {
                        viewButton.attr("href", filePath);
                        viewButton.attr("onclick", "viewRawData(event)");
                    }
                    viewButton.addClass("btn btn-primary btn-md");

                    // Create download button
                    var downloadButton = $("<a></a>").text("Download");
                    downloadButton.attr("href", filePath);
                    downloadButton.attr("download", "");
                    downloadButton.addClass("btn btn-primary btn-md");

                    // Append elements to list item
                    listItem.append(fileNameSpan);
                    listItem.append(viewButton);
                    listItem.append($("<span> | </span>"));
                    listItem.append(downloadButton);

                    // Append list item to list container
                    $("#objects-downloads").append(listItem);
                });
            }
        });
    }

});

function viewRawData(event) {

    event.preventDefault();

    const filePath = event.target.href;

    const xhr = new XMLHttpRequest();
    xhr.open('GET', filePath);
    xhr.setRequestHeader('Content-Type', 'text/plain');
    xhr.onload = function () {
        const rawFileContent = xhr.responseText;
        const newWindow = window.open('', '_blank');
        newWindow.document.open();
        newWindow.document.write('<pre>' + rawFileContent + '</pre>');
        newWindow.document.close();
    };
    xhr.send();
}

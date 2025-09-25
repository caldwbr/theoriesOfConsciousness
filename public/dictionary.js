document.addEventListener('DOMContentLoaded', () => {
    const filterButtons = document.querySelectorAll('.filter-btn');
    const dictionaryEntriesContainer = document.getElementById('dictionary-entries');
    const allEntries = Array.from(document.querySelectorAll('.entry'));

    // Function to sort entries alphabetically
    const sortEntries = (entries) => {
        entries.sort((a, b) => {
            const aText = a.querySelector('h3').textContent.trim();
            const bText = b.querySelector('h3').textContent.trim();
            return aText.localeCompare(bText);
        });
    };

    // Function to filter and display entries
    const filterAndDisplayEntries = (filterValue) => {
        // Sort all entries first
        sortEntries(allEntries);

        // Re-append sorted entries to the container in the correct order
        allEntries.forEach(entry => {
            dictionaryEntriesContainer.appendChild(entry);
        });

        // Loop through and hide/show based on the filter
        allEntries.forEach(entry => {
            if (filterValue === 'all' || entry.classList.contains(filterValue)) {
                entry.style.display = 'block';
            } else {
                entry.style.display = 'none';
            }
        });
    };

    // Initial sort and display on page load
    filterAndDisplayEntries('all');

    // Add event listeners to the filter buttons
    filterButtons.forEach(button => {
        button.addEventListener('click', () => {
            // Update the active button
            filterButtons.forEach(btn => btn.classList.remove('active'));
            button.classList.add('active');

            const filterValue = button.getAttribute('data-filter');
            
            // Run the main function to filter and display
            filterAndDisplayEntries(filterValue);
        });
    });
});